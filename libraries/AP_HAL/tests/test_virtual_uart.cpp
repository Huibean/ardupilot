#include <AP_gtest.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/VirtualUARTDriver.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#if AP_HAL_VIRTUAL_UART_ENABLED

TEST(VirtualUART, Registration)
{
    EXPECT_EQ(AP_HAL::VirtualUARTDriver::get_uart(0), nullptr);
    EXPECT_EQ(AP_HAL::VirtualUARTDriver::get_uart(3), nullptr);
    {
        AP_HAL::VirtualUARTDriver first(1);
        AP_HAL::VirtualUARTDriver uart(2);
        EXPECT_EQ(AP_HAL::VirtualUARTDriver::get_uart(1), &first);
        EXPECT_EQ(AP_HAL::VirtualUARTDriver::get_uart(2), &uart);
        AP_HAL::VirtualUARTDriver duplicate(2);
        duplicate.begin(57600);
        EXPECT_FALSE(duplicate.is_initialized());
        EXPECT_EQ(AP_HAL::VirtualUARTDriver::get_uart(2), &uart);
        AP_HAL::VirtualUARTDriver invalid(0);
        invalid.begin(57600);
        EXPECT_FALSE(invalid.is_initialized());
        AP_HAL::VirtualUARTDriver out_of_range(3);
        out_of_range.begin(57600);
        EXPECT_FALSE(out_of_range.is_initialized());
        EXPECT_EQ(AP_HAL::VirtualUARTDriver::get_uart(3), nullptr);
    }
    EXPECT_EQ(AP_HAL::VirtualUARTDriver::get_uart(1), nullptr);
    EXPECT_EQ(AP_HAL::VirtualUARTDriver::get_uart(2), nullptr);
}

TEST(VirtualUART, BidirectionalQueues)
{
    AP_HAL::VirtualUARTDriver uart(2);
    uint8_t output[600] {};
    uint8_t input[600];
    for (uint16_t i = 0; i < sizeof(input); i++) {
        input[i] = i;
    }
    EXPECT_FALSE(uart.is_initialized());
    EXPECT_EQ(uart.txspace(), 0U);
    EXPECT_EQ(uart.rxspace(), 0U);
    EXPECT_EQ(uart.write(input, sizeof(input)), 0U);
    EXPECT_EQ(uart.write_rx(input, sizeof(input)), 0U);

    uart.begin(57600);
    EXPECT_TRUE(uart.is_initialized());
    EXPECT_EQ(uart.get_baud_rate(), 57600U);
    EXPECT_EQ(uart.txspace(), 512U);
    EXPECT_EQ(uart.rxspace(), 512U);
    EXPECT_EQ(uart.write(input, sizeof(input)), 512U);
    EXPECT_TRUE(uart.tx_pending());
    EXPECT_EQ(uart.txspace(), 0U);
    EXPECT_EQ(uart.write(input, 1), 0U);
    // TX bytes must not leak into the frontend's RX queue.
    EXPECT_EQ(uart.available(), 0U);
    EXPECT_EQ(uart.read_tx(output, sizeof(output)), 512U);
    EXPECT_EQ(memcmp(input, output, 512), 0);
    EXPECT_FALSE(uart.tx_pending());

    EXPECT_EQ(uart.write_rx(input, sizeof(input)), 512U);
    EXPECT_EQ(uart.rxspace(), 0U);
    EXPECT_EQ(uart.write_rx(input, 1), 0U);
    EXPECT_EQ(uart.available(), 512U);
    EXPECT_EQ(uart.read(output, sizeof(output)), 512);
    EXPECT_EQ(memcmp(input, output, 512), 0);
    EXPECT_EQ(uart.available(), 0U);
    EXPECT_EQ(uart.read(), -1);
    EXPECT_EQ(uart.read_tx(output, sizeof(output)), 0U);
}

TEST(VirtualUART, WrappedQueuesSurviveBegin)
{
    AP_HAL::VirtualUARTDriver uart(2);
    uint8_t input[750];
    uint8_t output[750];
    for (uint16_t i = 0; i < sizeof(input); i++) {
        input[i] = i;
    }
    uart.begin(57600);
    ASSERT_EQ(uart.write(input, 350), 350U);
    ASSERT_EQ(uart.write_rx(input, 350), 350U);
    ASSERT_EQ(uart.read_tx(output, 300), 300U);
    ASSERT_EQ(uart.read(output, 300), 300);
    ASSERT_EQ(uart.write(input + 350, 400), 400U);
    ASSERT_EQ(uart.write_rx(input + 350, 400), 400U);

    uart.begin(115200, 1024, 2048);
    EXPECT_EQ(uart.get_baud_rate(), 115200U);
    EXPECT_EQ(uart.rxspace(), 1024U - 450U);
    EXPECT_EQ(uart.txspace(), 2048U - 450U);
    // Repeated begin and smaller requested minima must not erase or shrink.
    uart.begin(0);
    uart.begin(115200, 256, 256);
    EXPECT_EQ(uart.get_baud_rate(), 115200U);
    EXPECT_EQ(uart.rxspace(), 1024U - 450U);
    EXPECT_EQ(uart.txspace(), 2048U - 450U);
    EXPECT_EQ(uart.read_tx(output, sizeof(output)), 450U);
    EXPECT_EQ(memcmp(input + 300, output, 450), 0);
    EXPECT_EQ(uart.read(output, sizeof(output)), 450);
    EXPECT_EQ(memcmp(input + 300, output, 450), 0);
}

TEST(VirtualUART, LockAppliesOnlyToFrontend)
{
    AP_HAL::VirtualUARTDriver uart(2);
    const uint32_t key = 1234;
    const uint8_t input[] = {1, 2, 3, 4};
    uint8_t output[sizeof(input)] {};
    uart.begin(57600);
    ASSERT_TRUE(uart.lock_port(key, key));
    EXPECT_EQ(uart.write(input, sizeof(input)), 0U);
    EXPECT_EQ(uart.write_locked(input, sizeof(input), key + 1), 0U);
    EXPECT_EQ(uart.write_locked(input, sizeof(input), key), sizeof(input));
    EXPECT_EQ(uart.read_tx(output, sizeof(output)), sizeof(input));
    EXPECT_EQ(memcmp(input, output, sizeof(input)), 0);
    EXPECT_EQ(uart.write_rx(input, sizeof(input)), sizeof(input));
    EXPECT_EQ(uart.available(), 0U);
    EXPECT_EQ(uart.available_locked(key), sizeof(input));
    EXPECT_EQ(uart.read(output, sizeof(output)), 0);
    EXPECT_EQ(uart.read_locked(output, sizeof(output), key + 1), 0);
    EXPECT_FALSE(uart.discard_input());
    uart.begin(115200, 1024, 1024);
    EXPECT_EQ(uart.get_baud_rate(), 57600U);
    uart.begin_locked(115200, 1024, 1024, key);
    EXPECT_EQ(uart.get_baud_rate(), 115200U);
    EXPECT_EQ(uart.read_locked(output, sizeof(output), key), ssize_t(sizeof(input)));
    EXPECT_EQ(memcmp(input, output, sizeof(input)), 0);
    uart.end();
    EXPECT_TRUE(uart.is_initialized());
    ASSERT_TRUE(uart.lock_port(0, 0));
    uart.end();
    EXPECT_FALSE(uart.is_initialized());
}

TEST(VirtualUART, CapacityLimitAndLifecycle)
{
    AP_HAL::VirtualUARTDriver uart(2);
    uint8_t value = 42;
    uart.begin(57600, 4097, 512);
    EXPECT_FALSE(uart.is_initialized());
    uart.begin(57600, 4096, 4096);
    EXPECT_EQ(uart.rxspace(), 4096U);
    EXPECT_EQ(uart.txspace(), 4096U);
    EXPECT_EQ(uart.write(&value, 1), 1U);
    EXPECT_EQ(uart.write_rx(&value, 1), 1U);
    uart.begin(115200, 512, 65535);
    EXPECT_EQ(uart.get_baud_rate(), 57600U);
    EXPECT_EQ(uart.txspace(), 4095U);
    EXPECT_EQ(uart.available(), 1U);
    uart.flush();
    EXPECT_TRUE(uart.tx_pending());
    EXPECT_EQ(uart.available(), 1U);
    EXPECT_TRUE(uart.discard_input());
    EXPECT_EQ(uart.available(), 0U);
    EXPECT_TRUE(uart.tx_pending());
    uart.end();
    EXPECT_FALSE(uart.is_initialized());
    EXPECT_EQ(uart.get_baud_rate(), 0U);
    EXPECT_EQ(uart.txspace(), 0U);
    EXPECT_EQ(uart.rxspace(), 0U);
    uart.begin(38400);
    EXPECT_TRUE(uart.is_initialized());
    EXPECT_FALSE(uart.tx_pending());
    EXPECT_EQ(uart.available(), 0U);
    EXPECT_EQ(uart.rxspace(), 512U);
    EXPECT_EQ(uart.txspace(), 512U);
}

#if AP_UART_MONITOR_ENABLED
TEST(VirtualUART, FrontendReadMonitoring)
{
    AP_HAL::VirtualUARTDriver uart(2);
    ByteBuffer monitor(16);
    const uint8_t input[] = {4, 5, 6};
    uint8_t output[3] {};
    uart.begin(57600);
    uart.set_monitor_read_buffer(&monitor);
    EXPECT_EQ(uart.write(input, sizeof(input)), sizeof(input));
    EXPECT_EQ(uart.read_tx(output, sizeof(output)), sizeof(output));
    EXPECT_EQ(monitor.available(), 0U);
    EXPECT_EQ(uart.write_rx(input, sizeof(input)), sizeof(input));
    EXPECT_EQ(uart.read(output, sizeof(output)), ssize_t(sizeof(output)));
    EXPECT_EQ(monitor.read(output, sizeof(output)), sizeof(output));
    EXPECT_EQ(memcmp(input, output, sizeof(input)), 0);
    uart.set_monitor_read_buffer(nullptr);
}
#endif

#endif // AP_HAL_VIRTUAL_UART_ENABLED

AP_GTEST_MAIN()
