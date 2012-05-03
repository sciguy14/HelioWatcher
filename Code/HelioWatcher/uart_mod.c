/*
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * <joerg@FreeBSD.ORG> wrote this file.  As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy me a beer in return.        Joerg Wunsch
 * ----------------------------------------------------------------------------
 *
 * Stdio demo, UART implementation
 *
 * $Id: uart.c,v 1.1 2005/12/28 21:38:59 joerg_wunsch Exp $
 *
 * Mod for mega644 BRL Jan2009
 */


/* CPU frequency */
#define F_CPU 16000000UL

/* UART baud rate */
#define UART_BAUD_0  9600
#define UART_BAUD_1  4800 


#include <stdint.h>
#include <stdio.h>

#include <avr/io.h>

#include "uart_mod.h"

/*
 * Initialize the UART to 9600 Bd, tx/rx, 8N1.
 */
void
uart_init_0(void)
{
#if F_CPU < 2000000UL && defined(U2X)
  UCSR0A = _BV(U2X);             /* improve baud rate error by using 2x clk */
  UBRR0L = (F_CPU / (8UL * UART_BAUD_0)) - 1;
#else
  UBRR0L = (F_CPU / (16UL * UART_BAUD_0)) - 1;
#endif
  UCSR0B = _BV(TXEN0) | _BV(RXEN0); /* tx/rx enable */
}

void
uart_init_1(void)
{
#if F_CPU < 2000000UL && defined(U2X)
  UCSR1A = _BV(U2X);             /* improve baud rate error by using 2x clk */
  UBRR1L = (F_CPU / (8UL * UART_BAUD_1)) - 1;
#else
  UBRR1L = (F_CPU / (16UL * UART_BAUD_1)) - 1;
#endif
  UCSR1B = _BV(TXEN1) | _BV(RXEN1); /* tx/rx enable */
}

/*
 * Send character c down the UART Tx, wait until tx holding register
 * is empty.
 */
int
uart_putchar_0(char c, FILE *stream)
{

  if (c == '\a')
    {
      fputs("*ring*\n", stderr);
      return 0;
    }

  if (c == '\n')
    uart_putchar_0('\r', stream);
  loop_until_bit_is_set(UCSR0A, UDRE0);
  UDR0 = c;

  return 0;
}

int
uart_putchar_1(char c, FILE *stream)
{

  if (c == '\a')
    {
      fputs("*ring*\n", stderr);
      return 0;
    }

  if (c == '\n')
    uart_putchar_1('\r', stream);
  loop_until_bit_is_set(UCSR1A, UDRE1);
  UDR1 = c;

  return 0;
}

/*
 * Receive a character from the UART Rx.
 *
 * This features a simple line-editor that allows to delete and
 * re-edit the characters entered, until either CR or NL is entered.
 * Printable characters entered will be echoed using uart_putchar().
 *
 * Editing characters:
 *
 * . \b (BS) or \177 (DEL) delete the previous character
 * . ^u kills the entire input buffer
 * . ^w deletes the previous word
 * . ^r sends a CR, and then reprints the buffer
 * . \t will be replaced by a single space
 *
 * All other control characters will be ignored.
 *
 * The internal line buffer is RX_BUFSIZE (80) characters long, which
 * includes the terminating \n (but no terminating \0).  If the buffer
 * is full (i. e., at RX_BUFSIZE-1 characters in order to keep space for
 * the trailing \n), any further input attempts will send a \a to
 * uart_putchar() (BEL character), although line editing is still
 * allowed.
 *
 * Input errors while talking to the UART will cause an immediate
 * return of -1 (error indication).  Notably, this will be caused by a
 * framing error (e. g. serial line "break" condition), by an input
 * overrun, and by a parity error (if parity was enabled and automatic
 * parity recognition is supported by hardware).
 *
 * Successive calls to uart_getchar() will be satisfied from the
 * internal buffer until that buffer is emptied again.
 */
int
uart_getchar_0(FILE *stream)
{
  uint8_t c;
  char *cp, *cp2;
  static char b[RX_BUFSIZE];
  static char *rxp;

  if (rxp == 0)
    for (cp = b;;)
      {
	loop_until_bit_is_set(UCSR0A, RXC0);
	if (UCSR0A & _BV(FE0))
	  return _FDEV_EOF;
	if (UCSR0A & _BV(DOR0))
	  return _FDEV_ERR;
	c = UDR0;
	/* behaviour similar to Unix stty ICRNL */
	if (c == '\r')
	  c = '\n';
	if (c == '\n')
	  {
	    *cp = c;
	    uart_putchar_0(c, stream);
	    rxp = b;
	    break;
	  }
	else if (c == '\t')
	  c = ' ';

	if ((c >= (uint8_t)' ' && c <= (uint8_t)'\x7e') ||
	    c >= (uint8_t)'\xa0')
	  {
	    if (cp == b + RX_BUFSIZE - 1)
	      uart_putchar_0('\a', stream);
	    else
	      {
		*cp++ = c;
		uart_putchar_0(c, stream);
	      }
	    continue;
	  }

	switch (c)
	  {
	  case 'c' & 0x1f:
	    return -1;

	  case '\b':
	  case '\x7f':
	    if (cp > b)
	      {
		uart_putchar_0('\b', stream);
		uart_putchar_0(' ', stream);
		uart_putchar_0('\b', stream);
		cp--;
	      }
	    break;

	  case 'r' & 0x1f:
	    uart_putchar_0('\r', stream);
	    for (cp2 = b; cp2 < cp; cp2++)
	      uart_putchar_0(*cp2, stream);
	    break;

	  case 'u' & 0x1f:
	    while (cp > b)
	      {
		uart_putchar_0('\b', stream);
		uart_putchar_0(' ', stream);
		uart_putchar_0('\b', stream);
		cp--;
	      }
	    break;

	  case 'w' & 0x1f:
	    while (cp > b && cp[-1] != ' ')
	      {
		uart_putchar_0('\b', stream);
		uart_putchar_0(' ', stream);
		uart_putchar_0('\b', stream);
		cp--;
	      }
	    break;
	  }
      }

  c = *rxp++;
  if (c == '\n')
    rxp = 0;

  return c;
}

int
uart_getchar_1(FILE *stream)
{
  uint8_t c;
  char *cp, *cp2;
  static char b[RX_BUFSIZE];
  static char *rxp;

  if (rxp == 0)
    for (cp = b;;)
      {
	loop_until_bit_is_set(UCSR1A, RXC1);
	if (UCSR1A & _BV(FE1))
	  return _FDEV_EOF;
	if (UCSR1A & _BV(DOR1))
	  return _FDEV_ERR;
	c = UDR1;
	/* behaviour similar to Unix stty ICRNL */
	if (c == '\r')
	  c = '\n';
	if (c == '\n')
	  {
	    *cp = c;
	    uart_putchar_1(c, stream);
	    rxp = b;
	    break;
	  }
	else if (c == '\t')
	  c = ' ';

	if ((c >= (uint8_t)' ' && c <= (uint8_t)'\x7e') ||
	    c >= (uint8_t)'\xa0')
	  {
	    if (cp == b + RX_BUFSIZE - 1)
	      uart_putchar_1('\a', stream);
	    else
	      {
		*cp++ = c;
		uart_putchar_1(c, stream);
	      }
	    continue;
	  }

	switch (c)
	  {
	  case 'c' & 0x1f:
	    return -1;

	  case '\b':
	  case '\x7f':
	    if (cp > b)
	      {
		uart_putchar_1('\b', stream);
		uart_putchar_1(' ', stream);
		uart_putchar_1('\b', stream);
		cp--;
	      }
	    break;

	  case 'r' & 0x1f:
	    uart_putchar_1('\r', stream);
	    for (cp2 = b; cp2 < cp; cp2++)
	      uart_putchar_1(*cp2, stream);
	    break;

	  case 'u' & 0x1f:
	    while (cp > b)
	      {
		uart_putchar_1('\b', stream);
		uart_putchar_1(' ', stream);
		uart_putchar_1('\b', stream);
		cp--;
	      }
	    break;

	  case 'w' & 0x1f:
	    while (cp > b && cp[-1] != ' ')
	      {
		uart_putchar_1('\b', stream);
		uart_putchar_1(' ', stream);
		uart_putchar_1('\b', stream);
		cp--;
	      }
	    break;
	  }
      }

  c = *rxp++;
  if (c == '\n')
    rxp = 0;

  return c;
}
