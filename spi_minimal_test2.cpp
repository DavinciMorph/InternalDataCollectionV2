// Minimal 2-port SPI test with aggressive init for Port1 (8-device chain)
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
    uint8_t start_pin;
    uint8_t zero_tx[243];
    alignas(8) uint8_t xfer_storage[64];

    void init_xfer() {
        std::memset(zero_tx, 0, sizeof(zero_tx));
        std::memset(xfer_storage, 0, sizeof(xfer_storage));
        auto* xfer = reinterpret_cast<struct spi_ioc_transfer*>(xfer_storage);
        xfer->tx_buf = reinterpret_cast<uintptr_t>(zero_tx);
        xfer->speed_hz = 6000000;
        xfer->bits_per_word = 8;
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
        xfer.len = len;
        xfer.speed_hz = 6000000;
        xfer.bits_per_word = 8;
        return ::ioctl(spi_fd, SPI_IOC_MESSAGE(1), &xfer) >= 0;
    }

    void spi_cmd(uint8_t cmd) {
        uint8_t tx = cmd, rx = 0;
        transfer(&tx, &rx, 1);
        sleep_ms(1);
    }

    void init_simple() {
        // Simple init like Python test
        spi_cmd(0x06); sleep_ms(200);  // RESET
        for (int j = 0; j < 10; j++) { spi_cmd(0x11); sleep_ms(20); }
        sleep_ms(100);
        // Read ID
        uint8_t tx3[3] = {0x20, 0x00, 0x00}, rx3[3] = {};
        transfer(tx3, rx3, 3);
        std::printf("  Device ID: 0x%02X\n", rx3[2]);
        // Write registers (no verify)
        uint8_t tw[3], rw[3];
        tw[0]=0x43; tw[1]=0x00; tw[2]=0xE0; transfer(tw, rw, 3); sleep_ms(200);
        tw[0]=0x41; tw[2]=0x96; transfer(tw, rw, 3); sleep_ms(10);
        for (int ch=0; ch<8; ch++) { tw[0]=0x45+ch; tw[2]=0x60; transfer(tw, rw, 3); sleep_ms(10); }
        tw[0]=0x55; tw[2]=0x20; transfer(tw, rw, 3); sleep_ms(10);
    }

    void init_aggressive() {
        // Aggressive init like acquisition engine
        // Multiple RESETs
        for (int r = 0; r < 3; r++) {
            spi_cmd(0x06);
            sleep_ms(200);
            for (int j = 0; j < 10; j++) { spi_cmd(0x11); sleep_ms(10); }
            sleep_ms(50);
        }
        sleep_ms(100);

        // Verify SDATAC with LOFF test
        uint8_t tw[3], rw[3];
        bool sdatac_ok = false;
        for (int attempt = 0; attempt < 100; attempt++) {
            tw[0]=0x42; tw[1]=0x00; tw[2]=0xAA; transfer(tw, rw, 3);  // WREG LOFF 0xAA
            sleep_ms(50);
            tw[0]=0x22; tw[1]=0x00; tw[2]=0x00; transfer(tw, rw, 3);  // RREG LOFF
            sleep_ms(10);
            if (rw[2] == 0xAA) {
                sdatac_ok = true;
                std::printf("  SDATAC verified on attempt %d\n", attempt + 1);
                break;
            }
            for (int s = 0; s < 3; s++) { spi_cmd(0x11); sleep_ms(10); }
            sleep_ms(50);
        }
        if (!sdatac_ok) {
            std::printf("  SDATAC FAILED\n");
        }

        // Restore LOFF
        tw[0]=0x42; tw[1]=0x00; tw[2]=0x00; transfer(tw, rw, 3);
        sleep_ms(50);

        // Read ID
        tw[0]=0x20; tw[1]=0x00; tw[2]=0x00; transfer(tw, rw, 3);
        std::printf("  Device ID: 0x%02X\n", rw[2]);

        // Write registers WITH verify (retry up to 100 times)
        auto write_verify = [&](uint8_t reg_addr, uint8_t value, const char* name) -> bool {
            for (int a = 0; a < 100; a++) {
                tw[0]=0x40|reg_addr; tw[1]=0x00; tw[2]=value; transfer(tw, rw, 3);
                sleep_ms(10);
                tw[0]=0x20|reg_addr; tw[1]=0x00; tw[2]=0x00; transfer(tw, rw, 3);
                if (rw[2] == value) {
                    if (a > 0) std::printf("    %s verified on attempt %d\n", name, a+1);
                    return true;
                }
                sleep_ms(20);
            }
            std::printf("    %s FAILED (wrote 0x%02X, read 0x%02X)\n", name, value, rw[2]);
            return false;
        };

        write_verify(0x03, 0xE0, "CONFIG3");
        sleep_ms(200);
        write_verify(0x01, 0x96, "CONFIG1");
        write_verify(0x02, 0xC0, "CONFIG2");
        for (int ch = 0; ch < 8; ch++) {
            char name[16]; std::snprintf(name, 16, "CH%dSET", ch+1);
            write_verify(0x05 + ch, 0x60, name);
        }
        write_verify(0x15, 0x20, "MISC1");
    }

    bool read_samples(int i2c_fd, int count) {
        int valid = 0;
        for (int s = 0; s < count; s++) {
            // Wait for DRDY
            bool drdy = false;
            for (int w = 0; w < 1000; w++) {
                ioctl(i2c_fd, I2C_SLAVE, 0x20);
                uint8_t reg = 0x00;
                write(i2c_fd, &reg, 1);
                uint8_t val;
                read(i2c_fd, &val, 1);
                if (!(val & (1 << start_pin))) { drdy = true; break; }
            }
            if (!drdy) continue;

            int frame_size = num_devices * 27;
            uint8_t rx[243] = {};
            read_raw(rx, frame_size);

            bool ok = true;
            for (int d = 0; d < num_devices; d++) {
                if ((rx[d*27] & 0xF0) != 0xC0) { ok = false; break; }
            }
            if (s == 0) {
                std::printf("  %s: first bytes: %02X %02X %02X %02X %02X %02X [%s]\n",
                    name, rx[0], rx[1], rx[2], rx[3], rx[4], rx[5], ok ? "OK" : "FAIL");
            }
            if (ok) valid++;
        }
        return valid > 0;
    }
};

int main() {
    int i2c_fd = open("/dev/i2c-6", O_RDWR);
    if (i2c_fd < 0) { perror("i2c"); return 1; }

    PortTest ports[2] = {
        {"Port1", -1, 8, 0},
        {"Port5", -1, 4, 3},
    };
    const char* paths[2] = {"/dev/spidev0.0", "/dev/spidev4.0"};

    for (int i = 0; i < 2; i++) {
        ports[i].spi_fd = open(paths[i], O_RDWR);
        uint8_t mode = SPI_MODE_1; ioctl(ports[i].spi_fd, SPI_IOC_WR_MODE, &mode);
        uint8_t bits = 8; ioctl(ports[i].spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
        uint32_t speed = 6000000; ioctl(ports[i].spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
        ports[i].init_xfer();
    }

    // Force START LOW, configure pins
    ioctl(i2c_fd, I2C_SLAVE, 0x21);
    uint8_t buf[2] = {0x01, 0x80}; write(i2c_fd, buf, 2); sleep_ms(50);
    uint8_t cfg_buf[2];
    ioctl(i2c_fd, I2C_SLAVE, 0x21);
    uint8_t reg = 0x03; write(i2c_fd, &reg, 1);
    uint8_t cfg; read(i2c_fd, &cfg, 1);
    cfg &= ~(1<<0); cfg &= ~(1<<3);  // Output
    cfg_buf[0]=0x03; cfg_buf[1]=cfg; write(i2c_fd, cfg_buf, 2);

    ioctl(i2c_fd, I2C_SLAVE, 0x20);
    reg = 0x03; write(i2c_fd, &reg, 1);
    read(i2c_fd, &cfg, 1);
    cfg |= (1<<0); cfg |= (1<<3);  // Input
    cfg_buf[0]=0x03; cfg_buf[1]=cfg; write(i2c_fd, cfg_buf, 2);

    // === Test 1: Simple init on both ===
    std::printf("\n=== TEST 1: Simple init on both ports ===\n");
    for (int i = 0; i < 2; i++) {
        std::printf("Init %s (simple):\n", ports[i].name);
        ports[i].init_simple();
    }
    // RDATAC + START
    for (int i = 0; i < 2; i++) ports[i].spi_cmd(0x10);
    sleep_ms(50);
    ioctl(i2c_fd, I2C_SLAVE, 0x21);
    reg = 0x01; write(i2c_fd, &reg, 1);
    uint8_t out; read(i2c_fd, &out, 1);
    out |= (1<<0) | (1<<3);
    buf[0]=0x01; buf[1]=out; write(i2c_fd, buf, 2);
    sleep_ms(200);

    for (int i = 0; i < 2; i++) {
        int valid = 0;
        for (int s = 0; s < 10; s++) {
            bool drdy = false;
            for (int w = 0; w < 1000; w++) {
                ioctl(i2c_fd, I2C_SLAVE, 0x20);
                reg = 0x00; write(i2c_fd, &reg, 1);
                uint8_t val; read(i2c_fd, &val, 1);
                if (!(val & (1 << ports[i].start_pin))) { drdy = true; break; }
            }
            if (!drdy) continue;
            int frame_size = ports[i].num_devices * 27;
            uint8_t rx[243] = {};
            ports[i].read_raw(rx, frame_size);
            bool ok = true;
            for (int d = 0; d < ports[i].num_devices; d++) {
                if ((rx[d*27] & 0xF0) != 0xC0) { ok = false; break; }
            }
            if (s == 0) {
                std::printf("  %s: %02X %02X %02X %02X %02X %02X [%s]\n",
                    ports[i].name, rx[0],rx[1],rx[2],rx[3],rx[4],rx[5], ok?"OK":"FAIL");
            }
            if (ok) valid++;
        }
        std::printf("  %s: %d/10 valid\n", ports[i].name, valid);
    }

    // Cleanup
    ioctl(i2c_fd, I2C_SLAVE, 0x21);
    buf[0]=0x01; buf[1]=0x80; write(i2c_fd, buf, 2);
    for (int i = 0; i < 2; i++) {
        ports[i].spi_cmd(0x0A);
        for (int j = 0; j < 10; j++) ports[i].spi_cmd(0x11);
    }
    sleep_ms(100);

    // === Test 2: Aggressive init on Port1, simple on Port5 ===
    std::printf("\n=== TEST 2: Aggressive init on Port1, simple on Port5 ===\n");
    std::printf("Init Port1 (aggressive):\n");
    ports[0].init_aggressive();
    std::printf("Init Port5 (simple):\n");
    ports[1].init_simple();

    // RDATAC + START
    for (int i = 0; i < 2; i++) ports[i].spi_cmd(0x10);
    sleep_ms(50);
    ioctl(i2c_fd, I2C_SLAVE, 0x21);
    reg = 0x01; write(i2c_fd, &reg, 1);
    read(i2c_fd, &out, 1);
    out |= (1<<0) | (1<<3);
    buf[0]=0x01; buf[1]=out; write(i2c_fd, buf, 2);
    sleep_ms(200);

    for (int i = 0; i < 2; i++) {
        int valid = 0;
        for (int s = 0; s < 10; s++) {
            bool drdy = false;
            for (int w = 0; w < 1000; w++) {
                ioctl(i2c_fd, I2C_SLAVE, 0x20);
                reg = 0x00; write(i2c_fd, &reg, 1);
                uint8_t val; read(i2c_fd, &val, 1);
                if (!(val & (1 << ports[i].start_pin))) { drdy = true; break; }
            }
            if (!drdy) continue;
            int frame_size = ports[i].num_devices * 27;
            uint8_t rx[243] = {};
            ports[i].read_raw(rx, frame_size);
            bool ok = true;
            for (int d = 0; d < ports[i].num_devices; d++) {
                if ((rx[d*27] & 0xF0) != 0xC0) { ok = false; break; }
            }
            if (s == 0) {
                std::printf("  %s: %02X %02X %02X %02X %02X %02X [%s]\n",
                    ports[i].name, rx[0],rx[1],rx[2],rx[3],rx[4],rx[5], ok?"OK":"FAIL");
            }
            if (ok) valid++;
        }
        std::printf("  %s: %d/10 valid\n", ports[i].name, valid);
    }

    // Final cleanup
    ioctl(i2c_fd, I2C_SLAVE, 0x21);
    buf[0]=0x01; buf[1]=0x80; write(i2c_fd, buf, 2);
    for (int i = 0; i < 2; i++) {
        ports[i].spi_cmd(0x0A);
        for (int j = 0; j < 10; j++) ports[i].spi_cmd(0x11);
        close(ports[i].spi_fd);
    }
    close(i2c_fd);
    std::printf("\nDone\n");
    return 0;
}
