#include "update/Updater.h"

#include <array>
#include <cctype>
#include <cstdio>
#include <fstream>
#include <vector>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <winhttp.h>
#endif

namespace elem {
namespace {

#ifdef _WIN32

std::wstring toW(const std::string& s) {
    if (s.empty()) return {};
    const int n = MultiByteToWideChar(CP_UTF8, 0, s.c_str(), static_cast<int>(s.size()), nullptr, 0);
    std::wstring w(n, L'\0');
    MultiByteToWideChar(CP_UTF8, 0, s.c_str(), static_cast<int>(s.size()), w.data(), n);
    return w;
}

// GET a URL over HTTPS, following redirects, into `out`. Returns false on any
// transport error or a non-200 status, so a missing releases/latest (404 before
// the first release) simply reads as "no update". Handles both the small JSON
// API response and the binary release zip.
bool httpsGet(const std::string& url, std::string& out) {
    // Manual split -- these URLs are always https://host/path with no auth part.
    std::string u = url;
    if (u.rfind("https://", 0) == 0) u = u.substr(8);
    else if (u.rfind("http://", 0) == 0) u = u.substr(7);
    const std::size_t slash = u.find('/');
    const std::string host = u.substr(0, slash);
    const std::string path = slash == std::string::npos ? "/" : u.substr(slash);

    HINTERNET session = WinHttpOpen(L"Elemancer-Updater/1.0", WINHTTP_ACCESS_TYPE_DEFAULT_PROXY,
                                    WINHTTP_NO_PROXY_NAME, WINHTTP_NO_PROXY_BYPASS, 0);
    if (!session) return false;
    WinHttpSetTimeouts(session, 6000, 6000, 8000, 12000);

    HINTERNET connect =
        WinHttpConnect(session, toW(host).c_str(), INTERNET_DEFAULT_HTTPS_PORT, 0);
    HINTERNET request =
        connect ? WinHttpOpenRequest(connect, L"GET", toW(path).c_str(), nullptr,
                                     WINHTTP_NO_REFERER, WINHTTP_DEFAULT_ACCEPT_TYPES,
                                     WINHTTP_FLAG_SECURE)
                : nullptr;

    bool ok = false;
    if (request) {
        WinHttpAddRequestHeaders(request, L"Accept: application/vnd.github+json\r\n",
                                 static_cast<DWORD>(-1), WINHTTP_ADDREQ_FLAG_ADD);
        if (WinHttpSendRequest(request, WINHTTP_NO_ADDITIONAL_HEADERS, 0, WINHTTP_NO_REQUEST_DATA,
                               0, 0, 0) &&
            WinHttpReceiveResponse(request, nullptr)) {
            DWORD code = 0, codeLen = sizeof(code);
            WinHttpQueryHeaders(request,
                                WINHTTP_QUERY_STATUS_CODE | WINHTTP_QUERY_FLAG_NUMBER,
                                WINHTTP_HEADER_NAME_BY_INDEX, &code, &codeLen,
                                WINHTTP_NO_HEADER_INDEX);
            if (code == 200) {
                DWORD avail = 0;
                do {
                    if (!WinHttpQueryDataAvailable(request, &avail)) break;
                    if (avail == 0) break;
                    std::string chunk(avail, '\0');
                    DWORD read = 0;
                    if (!WinHttpReadData(request, chunk.data(), avail, &read)) break;
                    chunk.resize(read);
                    out += chunk;
                } while (avail > 0);
                ok = true;
            }
        }
    }

    if (request) WinHttpCloseHandle(request);
    if (connect) WinHttpCloseHandle(connect);
    WinHttpCloseHandle(session);
    return ok;
}

#else
bool httpsGet(const std::string&, std::string&) { return false; }
#endif

// The value of a top-level string field, by minimal scanning -- enough for the
// GitHub release JSON, which does not escape quotes or slashes in these fields.
std::string jsonString(const std::string& j, const std::string& key) {
    const std::string pat = "\"" + key + "\"";
    std::size_t p = j.find(pat);
    if (p == std::string::npos) return {};
    p = j.find(':', p + pat.size());
    if (p == std::string::npos) return {};
    p = j.find('"', p);
    if (p == std::string::npos) return {};
    const std::size_t q = j.find('"', p + 1);
    if (q == std::string::npos) return {};
    return j.substr(p + 1, q - p - 1);
}

// The first asset download URL ending in .zip (the win64 package).
std::string firstZipUrl(const std::string& j) {
    const std::string key = "\"browser_download_url\"";
    std::size_t p = 0;
    while ((p = j.find(key, p)) != std::string::npos) {
        std::size_t c = j.find(':', p + key.size());
        std::size_t a = c == std::string::npos ? std::string::npos : j.find('"', c);
        std::size_t b = a == std::string::npos ? std::string::npos : j.find('"', a + 1);
        if (b == std::string::npos) break;
        const std::string val = j.substr(a + 1, b - a - 1);
        if (val.size() >= 4 && val.compare(val.size() - 4, 4, ".zip") == 0) return val;
        p = b + 1;
    }
    return {};
}

// "v1.2.3" or "v1.2.3-4-gabc" -> {1,2,3}. A bare commit (no leading semver) maps
// to {0,0,0}, so an untagged build is treated as older than any real release.
std::array<int, 3> parseSemver(const std::string& s) {
    std::array<int, 3> v{0, 0, 0};
    std::size_t i = (!s.empty() && (s[0] == 'v' || s[0] == 'V')) ? 1 : 0;
    for (int part = 0; part < 3 && i < s.size(); ++part) {
        int val = 0;
        while (i < s.size() && std::isdigit(static_cast<unsigned char>(s[i]))) {
            val = val * 10 + (s[i] - '0');
            ++i;
        }
        v[part] = val;
        if (i < s.size() && s[i] == '.') ++i;
        else break;
    }
    return v;
}

bool isNewer(const std::string& remote, const std::string& local) {
    return parseSemver(remote) > parseSemver(local);
}

}  // namespace

Updater::~Updater() {
    if (thread_.joinable()) thread_.join();
}

void Updater::checkAsync(const std::string& repoSlug, const std::string& current,
                         const std::string& appDir) {
    {
        std::lock_guard<std::mutex> lock(mu_);
        appDir_ = appDir;
    }
    thread_ = std::thread([this, repoSlug, current]() {
        Status s;
        s.checked = true;
        std::string body;
        if (httpsGet("https://api.github.com/repos/" + repoSlug + "/releases/latest", body)) {
            const std::string tag = jsonString(body, "tag_name");
            const std::string url = firstZipUrl(body);
            if (!tag.empty() && !url.empty() && isNewer(tag, current)) {
                s.available = true;
                s.latestVersion = tag;
                s.downloadUrl = url;
            }
        }
        std::lock_guard<std::mutex> lock(mu_);
        status_ = s;
    });
}

Updater::Status Updater::status() const {
    std::lock_guard<std::mutex> lock(mu_);
    return status_;
}

#ifdef _WIN32

bool Updater::applyAndRestart(std::string& error) {
    Status s;
    std::string appDir;
    {
        std::lock_guard<std::mutex> lock(mu_);
        s = status_;
        appDir = appDir_;
    }
    if (!s.available || s.downloadUrl.empty()) {
        error = "no update to apply";
        return false;
    }

    // Download the zip into %TEMP%.
    std::string zip;
    if (!httpsGet(s.downloadUrl, zip) || zip.empty()) {
        error = "download failed";
        return false;
    }
    char tempBuf[MAX_PATH];
    const DWORD tn = GetTempPathA(MAX_PATH, tempBuf);
    if (tn == 0 || tn >= MAX_PATH) {
        error = "no temp path";
        return false;
    }
    const std::string temp = tempBuf;  // has a trailing backslash
    const std::string zipPath = temp + "elemancer_update.zip";
    const std::string extractDir = temp + "elemancer_update_x";
    const std::string batPath = temp + "elemancer_update.bat";

    std::ofstream zf(zipPath, std::ios::binary | std::ios::trunc);
    if (!zf) {
        error = "cannot write update zip";
        return false;
    }
    zf.write(zip.data(), static_cast<std::streamsize>(zip.size()));
    zf.close();

    // A running exe and its loaded DLLs cannot overwrite themselves, so hand the
    // swap to a detached helper that waits for this process to exit, unpacks the
    // zip (its top folder is "Elemancer"), copies it over the install, and
    // relaunches. robocopy's success codes are < 8; the batch ignores them.
    std::ofstream bf(batPath, std::ios::trunc);
    if (!bf) {
        error = "cannot write updater script";
        return false;
    }
    bf << "@echo off\r\n"
       << "setlocal\r\n"
       << "set PID=" << GetCurrentProcessId() << "\r\n"
       << ":wait\r\n"
       << "tasklist /FI \"PID eq %PID%\" 2>nul | find \"%PID%\" >nul\r\n"
       << "if not errorlevel 1 ( timeout /t 1 /nobreak >nul & goto wait )\r\n"
       << "rmdir /s /q \"" << extractDir << "\" 2>nul\r\n"
       << "powershell -NoProfile -Command \"Expand-Archive -Path '" << zipPath
       << "' -DestinationPath '" << extractDir << "' -Force\"\r\n"
       << "robocopy \"" << extractDir << "\\Elemancer\" \"" << appDir
       << "\" /E /NFL /NDL /NJH /NJS /R:3 /W:1 >nul\r\n"
       << "start \"\" \"" << appDir << "\\elemancer.exe\"\r\n"
       << "rmdir /s /q \"" << extractDir << "\" 2>nul\r\n"
       << "del /q \"" << zipPath << "\" 2>nul\r\n"
       << "del /q \"%~f0\"\r\n";
    bf.close();

    std::string cmd = "cmd.exe /c \"" + batPath + "\"";
    std::vector<char> cmdBuf(cmd.begin(), cmd.end());
    cmdBuf.push_back('\0');
    STARTUPINFOA si{};
    si.cb = sizeof(si);
    PROCESS_INFORMATION pi{};
    const BOOL launched =
        CreateProcessA(nullptr, cmdBuf.data(), nullptr, nullptr, FALSE,
                       CREATE_NO_WINDOW | DETACHED_PROCESS | CREATE_NEW_PROCESS_GROUP, nullptr,
                       temp.c_str(), &si, &pi);
    if (!launched) {
        error = "could not launch updater";
        return false;
    }
    CloseHandle(pi.hThread);
    CloseHandle(pi.hProcess);
    return true;  // caller should now exit so the helper can replace the files
}

#else

bool Updater::applyAndRestart(std::string& error) {
    error = "updates are only supported on Windows";
    return false;
}

#endif

}  // namespace elem
