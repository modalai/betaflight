
#include "drivers/serial.h"

extern int sl_client_register_uart_callback(int fd, serialReceiveCallbackPtr cb, void *arg);
extern int sl_client_config_uart(uint8_t port_number, uint32_t speed);
extern int sl_client_disable_uart_tx_wait(int fd);
extern int sl_client_uart_read(int fd, char *buffer, const unsigned buffer_len);
extern int sl_client_uart_flush_rx(int fd);
extern int sl_client_uart_rx_available(int fd, uint32_t *data);
extern int sl_client_uart_write(int fd, const char *data, const unsigned data_len);

// Call this to send a message to host side
extern int sl_client_send_data(const uint8_t *data, int data_len_in_bytes);

extern int sl_client_config_spi_bus(void);
extern int sl_client_spi_transfer(int fd, const uint8_t *send, uint8_t *recv, const unsigned len);

extern int sl_client_register_interrupt_callback(int (*func)(int, void*, void*), void* arg);
