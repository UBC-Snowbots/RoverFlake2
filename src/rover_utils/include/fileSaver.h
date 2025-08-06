#include <fstream>
#include <filesystem>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <stdexcept>

class NonVolatileMemory {
public:
    // If dir_path is omitted, use $HOME/RoverLogs/default_folder
    explicit NonVolatileMemory(
        const std::string& file_name,
        const std::string& roverlog_path = "default_folder/",
        const std::string& dir_path = std::string{std::getenv("HOME")} + "/Logs_Rover/"
    )
      : file_path_{dir_path + "/" + file_name},
        ofs_{}
    {
        namespace fs = std::filesystem;
        if (!fs::exists(dir_path)) {
            fs::create_directories(dir_path);
        }

        ofs_.open(file_path_, std::ios::app | std::ios::binary);
        if (!ofs_) {
            throw std::runtime_error("NonVolatileMemory: failed to open " + file_path_);
        }
    }

    ~NonVolatileMemory() {
        ofs_.close();
    }

    // Raw write (no newline, no timestamp)
    void write(const std::string& data) {
        ofs_ << data;
        ofs_.flush();    // uncomment if you need each write to hit disk immediately
    }

    // Write + newline
    void writeLine(const std::string& data = "") {
        ofs_ << data << '\n';
        ofs_.flush();
    }

    // Prepend an ISO-style timestamp (YYYY-MM-DD HH:MM:SS) and newline
    void writeStamped(const std::string& data) {
        auto now = std::chrono::system_clock::now();
        auto tt  = std::chrono::system_clock::to_time_t(now);
        std::tm  tm;
        localtime_r(&tt, &tm);  // thread-safe

        // e.g. "2025-08-05 22:45:03"
        char buf[32];
        std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tm);

        ofs_ << buf << " " << data << '\n';
    }

    // Read entire file into a std::string
    std::string readAll() const {
        std::ifstream ifs{file_path_, std::ios::binary | std::ios::ate};
        if (!ifs) return {};

        auto sz = ifs.tellg();
        std::string content;
        content.resize(sz);

        ifs.seekg(0);
        ifs.read(&content[0], sz);
        return content;
    }

private:
    std::string    file_path_;
    std::ofstream  ofs_;
};