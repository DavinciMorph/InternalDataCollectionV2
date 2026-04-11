// Minimal SPI test using the acquisition engine's SPIBus class
// Tests two ports simultaneously (SPI0.0 and SPI4.0)
// Mimics the Python test's simple init sequence

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

struct PortTest {
    const char* name;
    int spi_fd;
    int num_devices;
    uint8_t start_pin;  // TCA9534 pin number

    // Pre-allocated zero TX buffer (matches SPIBus::zero_tx_)
    uint8_t zero_tx[243];

    // Pre-initialized transfer struct (matches SPIBus::data_xfer_storage_)
    alignas(8) uint8_t xfer_storage[64];

    void init_xfer() {
        std::memset(zero_tx, 0, sizeof(zero_tx));
        std::memset(xfer_storage, 0, sizeof(xfer_storage));
        auto* xfer = reinterpret_cast<struct spi_ioc_transfer*>(xfer_storage);
        xfer->tx_buf = reinterpret_cast<uintptr_t>(zero_tx);
        xfer->speed_hz = 6000000;
        xfer->bits_per_word = 8;
        xfer->delay_usecs = 0;
    }

    bool read_raw(uint8_t* rx_buf, int len) {
        auto* xfer = reinterpret_cast<struct spi_ioc_transfer*>(xfer_storage);
        xfer->rx_buf = reinterpret_cast<uintptr_t>(rx_buf);
        xfer->len = static_cast<uint32_t>(len);
        return ::ioctl(spi_fd, SPI_IOC_MESSAGE(1), xfer) >= 0;
    }

    bool transfer(const uint8_t* tx, uint8_t* rx, int len) {
        struct spi_ioc_transfer xfer{};
        xfer.tx_buf = reinterpret_cast<uintptr_t>(tx);
        xfer.rx_buf = reinterpret_cast<uintptr_t>(rx);
        xfer.len = static_cast<uint32_t>(len);
        xfer.speed_hz = 6000000;
        xfer.bits_per_word = 8;
        return ::ioctl(spi_fd, SPI_IOC_MESSAGE(1), &xfer) >= 0;
    }

    void spi_cmd(uint8_t cmd) {
        uint8_t tx = cmd, rx = 0;
        transfer(&tx, &rx, 1);
        sleep_ms(1);
    }
};

int main() {
    int i2c_fd = open("/dev/i2c-6", O_RDWR);
    if (i2c_fd < 0) { perror("i2c"); return 1; }

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

    // Two ports to test
    PortTest ports[2] = {
        {"Port1", -1, 8, 0},   // SPI0.0, 8 devices, TCA9534 pin 0
        {"Port5", -1, 4, 3},   // SPI4.0, 4 devices, TCA9534 pin 3
    };

    const char* spi_paths[2] = {"/dev/spidev0.0", "/dev/spidev4.0"};

    // Open SPI devices
    for (int i = 0; i < 2; i++) {
        ports[i].spi_fd = open(spi_paths[i], O_RDWR);
        if (ports[i].spi_fd < 0) { perror(spi_paths[i]); return 1; }
        uint8_t mode = SPI_MODE_1;
        ioctl(ports[i].spi_fd, SPI_IOC_WR_MODE, &mode);
        uint8_t bits = 8;
        ioctl(ports[i].spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
        uint32_t speed = 6000000;
        ioctl(ports[i].spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
        ports[i].init_xfer();
        std::printf("Opened %s for %s (%d devices)\n", spi_paths[i], ports[i].name, ports[i].num_devices);
    }

    // Force all START pins LOW
    uint8_t out = i2c_read(0x21, 0x01);
    out &= 0x80;  // Keep pin 7 (unused), clear 0-6
    i2c_write(0x21, 0x01, out);
    sleep_ms(50);

    // Configure START pins as outputs
    uint8_t cfg = i2c_read(0x21, 0x03);
    for (int i = 0; i < 2; i++) {
        cfg &= ~(1 << ports[i].start_pin);
    }
    i2c_write(0x21, 0x03, cfg);

    // Configure DRDY pins as inputs
    cfg = i2c_read(0x20, 0x03);
    for (int i = 0; i < 2; i++) {
        cfg |= (1 << ports[i].start_pin);  // Same pin for DRDY and START
    }
    i2c_write(0x20, 0x03, cfg);

    // Init both ports - simple sequence like Python test
    for (int i = 0; i < 2; i++) {
        auto& p = ports[i];
        std::printf("\n--- Init %s ---\n", p.name);

        // RESET
        p.spi_cmd(0x06);
        sleep_ms(200);

        // SDATAC x10
        for (int j = 0; j < 10; j++) {
            p.spi_cmd(0x11);
            sleep_ms(20);
        }
        sleep_ms(100);

        // Read ID
        uint8_t tx3[3] = {0x20, 0x00, 0x00};
        uint8_t rx3[3] = {};
        p.transfer(tx3, rx3, 3);
        std::printf("  Device ID: 0x%02X\n", rx3[2]);

        // Write CONFIG3 = 0xE0
        uint8_t tw[3], rw[3];
        tw[0] = 0x43; tw[1] = 0x00; tw[2] = 0xE0;
        p.transfer(tw, rw, 3);
        sleep_ms(200);

        // Write CONFIG1 = 0x96
        tw[0] = 0x41; tw[2] = 0x96;
        p.transfer(tw, rw, 3);
        sleep_ms(10);

        // Write CHnSET = 0x60
        for (int ch = 0; ch < 8; ch++) {
            tw[0] = 0x40 + 0x05 + ch; tw[2] = 0x60;
            p.transfer(tw, rw, 3);
            sleep_ms(10);
        }

        // Write MISC1 = 0x20
        tw[0] = 0x55; tw[2] = 0x20;
        p.transfer(tw, rw, 3);
        sleep_ms(10);

        std::printf("  %s configured\n", p.name);
    }

    // Enter RDATAC on both
    std::printf("\n--- RDATAC on both ---\n");
    for (int i = 0; i < 2; i++) {
        ports[i].spi_cmd(0x10);
    }
    sleep_ms(50);

    // Assert START on both simultaneously
    std::printf("--- Asserting START on both ---\n");
    out = i2c_read(0x21, 0x01);
    for (int i = 0; i < 2; i++) {
        out |= (1 << ports[i].start_pin);
    }
    i2c_write(0x21, 0x01, out);
    sleep_ms(200);

    // Read 10 samples from each port
    std::printf("\n--- Reading samples ---\n");
    int valid_count[2] = {};

    for (int s = 0; s < 10; s++) {
        // Wait for any DRDY
        for (int w = 0; w < 1000; w++) {
            uint8_t drdy_val = i2c_read(0x20, 0x00);
            bool any_drdy = false;
            for (int i = 0; i < 2; i++) {
                if (!(drdy_val & (1 << ports[i].start_pin))) {
                    any_drdy = true;
                }
            }
            if (any_drdy) break;
        }

        // Read from both ports using read_raw (pre-initialized struct)
        for (int i = 0; i < 2; i++) {
            auto& p = ports[i];
            int frame_size = p.num_devices * 27;
            uint8_t rx_data[243] = {};
            bool raw_ok = p.read_raw(rx_data, frame_size);

            bool valid = true;
            for (int d = 0; d < p.num_devices; d++) {
                int offset = d * 27;
                uint8_t s0 = rx_data[offset];
                if ((s0 & 0xF0) != 0xC0) {
                    valid = false;
                    break;
                }
            }

            if (s == 0) {
                std::printf("  %s sample 0: raw_ok=%d, first 6 bytes: %02X %02X %02X %02X %02X %02X [%s]\n",
                    p.name, raw_ok,
                    rx_data[0], rx_data[1], rx_data[2],
                    rx_data[3], rx_data[4], rx_data[5],
                    valid ? "VALID" : "FAIL");
            }

            if (valid) valid_count[i]++;
        }
    }

    for (int i = 0; i < 2; i++) {
        std::printf("%s: %d/10 valid\n", ports[i].name, valid_count[i]);
    }

    // Cleanup
    out = i2c_read(0x21, 0x01);
    for (int i = 0; i < 2; i++) {
        out &= ~(1 << ports[i].start_pin);
    }
    i2c_write(0x21, 0x01, out);

    for (int i = 0; i < 2; i++) {
        ports[i].spi_cmd(0x0A);  // STOP
        for (int j = 0; j < 10; j++) {
            ports[i].spi_cmd(0x11);  // SDATAC
        }
        close(ports[i].spi_fd);
    }

    close(i2c_fd);
    std::printf("\nDone\n");
    return 0;
}
