#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <thread>

namespace elem {

// Checks GitHub Releases for a newer build and, on request, downloads it,
// swaps it in over the running install, and relaunches. Windows-only (WinHTTP);
// elsewhere every method is a no-op that reports "no update".
//
// The check runs on a background thread so a slow or absent network never
// stalls startup. Applying the update hands the file-replacement off to a small
// detached helper script, because a running exe (and its loaded DLLs) cannot
// overwrite itself -- the helper waits for this process to exit first.
class Updater {
public:
    struct Status {
        bool checked = false;      // the check finished (success or graceful failure)
        bool available = false;    // a newer release exists
        std::string latestVersion; // its tag, e.g. "v0.2.0"
        std::string downloadUrl;   // browser_download_url of its win64 zip
    };

    Updater() = default;
    ~Updater();
    Updater(const Updater&) = delete;
    Updater& operator=(const Updater&) = delete;

    // Kick off the check in the background. repoSlug is "owner/name"; current is
    // the baked-in version string; appDir is the folder holding elemancer.exe.
    void checkAsync(const std::string& repoSlug, const std::string& current,
                    const std::string& appDir);

    // Thread-safe snapshot of the check result.
    Status status() const;

    // Download the update, stage the replace+relaunch, and return true meaning
    // "the caller should now exit so the helper can take over". Returns false and
    // fills `error` if the download or staging failed. Blocking; call it from the
    // UI thread when the user clicks Update.
    bool applyAndRestart(std::string& error);

private:
    mutable std::mutex mu_;
    Status status_;
    std::string appDir_;
    std::thread thread_;
};

}  // namespace elem
