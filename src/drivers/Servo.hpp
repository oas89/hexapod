#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <termios.h>
#include <string>
#include <stdexcept>


class SerialServoController {
public:
    SerialServoController(const std::string& name)
    {
        fd = open(name.c_str(), O_RDWR | O_NOCTTY);
        if (fd < 0) {
            throw std::runtime_error("Cannot open device: " + name);
        }

        struct termios options;
	    tcgetattr(fd, &options);
	    options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	    options.c_oflag &= ~(ONLCR | OCRNL);
	    tcsetattr(fd, TCSANOW, &options);
    }

    ~SerialServoController()
    {
        close(fd);
    }

    bool set(uint8_t channel, uint8_t target)
    {
        uint8_t buffer[] = {0xFF, channel, target};
        ssize_t ret = write(fd, buffer, sizeof(buffer));
        return ret < 0 ? false : true;
    }

private:
    int fd;
};