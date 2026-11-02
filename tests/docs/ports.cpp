/**
 * @file ports.cpp
 * @brief Instantiates the documented SPI and UART ports with the driver, so both compile in full.
 */
#include "my_uart.hpp"

template class tmc9660::TMC9660<MySpi>;
template class tmc9660::TMC9660<MyUart>;
