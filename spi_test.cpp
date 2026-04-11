#include <cstdio>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <linux/i2c-dev.h>
#include <time.h>

static void sleep_ms(int ms) {
    struct timespec ts = {ms / 1000, (ms % 1000) * 1000000L};
    nanosleep(&ts, nullptr);
}

int main() {
    int i2c_fd = open("/dev/i2c-6", O_RDWR);
    if (i2c_fd < 0) { perror("i2c"); return 1; }

    int spi_fd = open("/dev/spidev4.0", O_RDWR);
    if (spi_fd < 0) { perror("spi"); return 1; }

    uint8_t mode = SPI_MODE_1;
    ioctl(spi_fd, SPI_IOC_WR_MODE, &mode);
    uint8_t bits = 8;
    ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    uint32_t speed = 6000000;
    ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

    auto spi_xfer = [&](uint8_t* tx, uint8_t* rx, int len) {
        struct spi_ioc_transfer xfer{};
        xfer.tx_buf = (uintptr_t)tx;
        xfer.rx_buf = (uintptr_t)rx;
        xfer.len = len;
        xfer.speed_hz = 6000000;
        xfer.bits_per_word = 8;
        return ioctl(spi_fd, SPI_IOC_MESSAGE(1), &xfer) >= 0;
    };

    auto spi_cmd = [&](uint8_t cmd) {
        uint8_t tx = cmd, rx = 0;
        spi_xfer(&tx, &rx, 1);
        sleep_ms(1);
    };

    auto i2c_write = [&](uint8_t addr, uint8_t reg, uint8_t val) {
        ioctl(i2c_fd, I2C_SLAVE, addr);
        uint8_t buf[2] = {reg, val};
        write(i2c_fd, buf, 2);
    };

    auto i2c_read = [&](uint8_t addr, uint8_t reg) -> uint8_t {
        ioctl(i2c_fd, I2C_SLAVE, addr);
        write(i2c_fd, &reg, 1);
        uint8_t val;
        read(i2c_fd, &val, 1);
        return val;
    };

    // Configure TCA9534 START pin for Port5 (pin 3) as output
    uint8_t cfg = i2c_read(0x21, 0x03);
    cfg &= ~(1 << 3);
    i2c_write(0x21, 0x03, cfg);

    // Configure TCA9534 DRDY pin for Port5 (pin 3) as input
    cfg = i2c_read(0x20, 0x03);
    cfg |= (1 << 3);
    i2c_write(0x20, 0x03, cfg);

    // Ensure START is LOW
    uint8_t out = i2c_read(0x21, 0x01);
    out &= ~(1 << 3);
    i2c_write(0x21, 0x01, out);
    sleep_ms(50);

    // RESET
    spi_cmd(0x06);
    sleep_ms(200);

    // SDATAC x10
    for (int i = 0; i < 10; i++) {
        spi_cmd(0x11);
        sleep_ms(20);
    }
    sleep_ms(100);

    // Read ID
    uint8_t tx3[3] = {0x20, 0x00, 0x00};
    uint8_t rx3[3] = {};
    spi_xfer(tx3, rx3, 3);
    std::printf("Device ID: 0x%02X\n", rx3[2]);

    // Write CONFIG3 = 0xE0
    uint8_t tw[3], rw[3];
    tw[0] = 0x43; tw[1] = 0x00; tw[2] = 0xE0;
    spi_xfer(tw, rw, 3);
    sleep_ms(200);

    // Write CONFIG1 = 0x96
    tw[0] = 0x41; tw[2] = 0x96;
    spi_xfer(tw, rw, 3);
    sleep_ms(10);

    // Write CHnSET = 0x60
    for (int ch = 0; ch < 8; ch++) {
        tw[0] = 0x40 + 0x05 + ch; tw[2] = 0x60;
        spi_xfer(tw, rw, 3);
        sleep_ms(10);
    }

    // Write MISC1 = 0x20
    tw[0] = 0x55; tw[2] = 0x20;
    spi_xfer(tw, rw, 3);
    sleep_ms(10);

    // RDATAC
    spi_cmd(0x10);
    sleep_ms(50);

    // Assert START via TCA9534
    out = i2c_read(0x21, 0x01);
    out |= (1 << 3);
    i2c_write(0x21, 0x01, out);
    std::printf("START asserted\n");
    sleep_ms(200);

    // Read 10 samples
    int num_devices = 4;
    int frame_size = num_devices * 27;
    int valid_count = 0;

    for (int s = 0; s < 10; s++) {
        bool drdy_got = false;
        int wait_count = 0;
        for (int w = 0; w < 1000; w++) {
            uint8_t drdy_val = i2c_read(0x20, 0x00);
            if (!(drdy_val & (1 << 3))) {
                drdy_got = true;
                wait_count = w;
                break;
            }
        }

        if (!drdy_got) {
            std::printf("Sample %d: DRDY timeout\n", s);
            continue;
        }

        uint8_t tx_zeros[108] = {};
        uint8_t rx_data[108] = {};
        spi_xfer(tx_zeros, rx_data, frame_size);

        bool valid = true;
        for (int d = 0; d < num_devices; d++) {
            int offset = d * 27;
            uint8_t s0 = rx_data[offset];
            if ((s0 & 0xF0) != 0xC0) {
                valid = false;
            }
            std::printf("  Dev%d: 0x%02X 0x%02X 0x%02X %s\n",
                   d, rx_data[offset], rx_data[offset+1], rx_data[offset+2],
                   ((s0 & 0xF0) == 0xC0) ? "[OK]" : "[FAIL]");
        }
        std::printf("Sample %d (drdy_wait=%d): %s\n", s, wait_count, valid ? "VALID" : "INVALID");
        if (valid) valid_count++;
    }

    std::printf("\nResult: %d/10 valid\n", valid_count);

    // Cleanup
    out = i2c_read(0x21, 0x01);
    out &= ~(1 << 3);
    i2c_write(0x21, 0x01, out);
    spi_cmd(0x0A);
    spi_cmd(0x11);

    close(spi_fd);
    close(i2c_fd);
    return 0;
}
