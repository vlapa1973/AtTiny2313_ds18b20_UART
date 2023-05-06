/*
    Выносной датчик температуры для котла
    спим 8 сек х N + 1 раз
    = vlapa =  20221130-20221205
    v.004
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
// #include <string.h>
#include <avr/wdt.h>   // здесь организована работа с ватчдогом
#include <avr/sleep.h> // здесь описаны режимы сна

#define Fclk_CPU 8000000UL
#define BAUDRATE 9600UL

// Датчик DS18B20 - любая свободная ножка МК
#define DS18B20 6
#define DS18B20_PORT PORTD
#define DS18B20_PIN PIND
#define DS18B20_DDR DDRD

#define pinSwitch 0

const uint8_t count = 7;  //  Кол-во циклов по ~ 8сек
uint8_t countWork = 0;

uint8_t sensorNumRoom = 99; 

uint8_t DS_scratchpad[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0}; // 9 байт, считанных с DS18B20,
// uint8_t Presense_errors = 0;                            // Счётчик ошибок - инициализация DS18B20
// uint8_t Short_circuit_errors = 0;                       // Счётчик ошибок - КЗ линии данных DS18B20
float Temperature = 0;                                  // Температура преобразованная (целая часть градусов)

//================================================================
//  Прерывание сторожевого таймера
ISR(WDT_OVERFLOW_vect)
{
  WDTCSR |= (1 << WDIE); // разрешаем прерывания по ватчдогу.
                         // Иначе будет резет.
  --countWork;
}

//================================================================
// Инициализирую DS18B20
void DS18B20_init(void)
{
  // if ((DS18B20_PIN & (1 << DS18B20)) == 0)
  //   Short_circuit_errors++;        // Проверяю КЗ линии данных
  DS18B20_PORT &= ~(1 << DS18B20); // Устанавливаю низкий уровень
  DS18B20_DDR |= (1 << DS18B20);
  _delay_us(490);
  DS18B20_DDR &= ~(1 << DS18B20);
  _delay_us(68);
  // if ((DS18B20_PIN & (1 << DS18B20)) > 0)
  //   Presense_errors++; // Ловлю импульс присутствия датчика
  // Если датчик не подключен, Presense_errors увеличиваю на 1
  _delay_us(422);
}

//================================================================
// Функция чтения байта из DS18B20
uint8_t DS18B20_read(void)
{
  uint8_t dat = 0;
  for (uint8_t i = 0; i < 8; i++)
  {
    DS18B20_DDR |= (1 << DS18B20);
    _delay_us(2);
    DS18B20_DDR &= ~(1 << DS18B20);
    _delay_us(4);
    dat = dat >> 1;
    if (DS18B20_PIN & (1 << DS18B20))
    {
      dat |= 0x80;
    }
    _delay_us(62);
  }
  return dat;
}

//================================================================
// Функция чтения "блокнота" из DS18B20
void DS18B20_read_scratchpad(void)
{
  for (uint8_t i = 0; i < 9; i++) // Считываю 9 байт данных, или так называемый "блокнот"
  {
    DS_scratchpad[i] = DS18B20_read();
  }
}

//================================================================
// Функция записи байта в DS18B20
void DS18B20_write(uint8_t dat)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    DS18B20_DDR |= (1 << DS18B20);
    _delay_us(2);
    if (dat & 0x01)
    {
      DS18B20_DDR &= ~(1 << DS18B20);
    }
    else
    {
      DS18B20_DDR |= (1 << DS18B20);
    }
    dat = dat >> 1;
    _delay_us(62);
    DS18B20_DDR &= ~(1 << DS18B20);
    _delay_us(2);
  }
}

//================================================================
// Reverses a string 'str' of length 'len'
void reverse(char *str, int len)
{
  int i = 0, j = len - 1, temp;
  while (i < j)
  {
    temp = str[i];
    str[i] = str[j];
    str[j] = temp;
    i++;
    j--;
  }
}

// Converts a given integer x to string str[].
// d is the number of digits required in the output.
// If d is more than the number of digits in x,
// then 0s are added at the beginning.
int intToStr(int x, char str[], int d)
{
  int i = 0;
  while (x)
  {
    str[i++] = (x % 10) + '0';
    x = x / 10;
  }

  // If number of digits required is more, then
  // add 0s at the beginning
  while (i < d)
    str[i++] = '0';

  reverse(str, i);
  str[i] = '\0';
  return i;
}

void ftoa(float n, char *res, int afterpoint)
{
  // Extract integer part
  int ipart = (int)n;

  // Extract floating part
  float fpart = n - (float)ipart;

  // convert integer part to string
  int i = intToStr(ipart, res, 0);

  // check for display option after point
  if (afterpoint != 0)
  {
    res[i] = '.'; // add dot

    // Get the value of fraction part upto given no.
    // of points after dot. The third parameter
    // is needed to handle cases like 233.007
    fpart = fpart * pow(10, afterpoint);

    intToStr((int)fpart, res + i + 1, afterpoint);
  }
}

//================================================================
//  Функция инициализации USART
void uart_init(void)
{
  // Параметры соединения: 8 бит данные, 1 стоповый бит, нет контроля четности
  UBRRL = (Fclk_CPU / BAUDRATE / 16 - 1); // Вычисляем скорость обмена данными
  UBRRH = (Fclk_CPU / BAUDRATE / 16 - 1) >> 8;
  UCSRB |= //(1 << RXCIE) | // Разрешаем прерывание по завершению приема данных
           //(1 << RXEN) | 
           (1 << TXEN); // Включаем приемник и передатчик
  UCSRC |= (1 << UCSZ1) | (1 << UCSZ0);
  // Для доступа к регистру UCSRC выставляем бит URSEL
  // Размер посылки в кадре 8 бит
}

//===========================================================
// Функция передачи данных по USART
void uart_send(char data)
{
  while (!(UCSRA & (1 << UDRE)))
    ;
  // Ожидаем когда очистится буфер передачи
  UDR = data; // Помещаем данные в буфер, начинаем передачу
}

// Функция передачи строки по USART
void str_uart_send(char *string)
{
  uart_send(sensorNumRoom / 10 + 0x30);
  uart_send(sensorNumRoom % 10 + 0x30);
  uart_send('%');
  while (*string != '\0')
  {
    uart_send(*string);
    string++;
  }
  uart_send(';');
  uart_send('\n');
}

// Функция приема данных по USART
int uart_receive(void)
{
  while (!(UCSRA & (1 << RXC)))
    ;         // Ожидаем, когда данные будут получены
  return UDR; // Читаем данные из буфера и возвращаем их при выходе из подпрограммы
}

//===========================================================

int main(void)
{
  //инициализация ватчдога
  wdt_reset();           // сбрасываем
  wdt_enable(WDTO_8S);   // разрешаем ватчдог 8 сек
  WDTCSR |= (1 << WDIE); // разрешаем прерывания по ватчдогу. Иначе будет резет.
  
  _delay_ms(1000);
  uart_init();

  DDRB = 0b11111111;
  PORTB = 0b00000000;

  sei();                 // разрешаем прерывания

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);

  while (1)
  {
    if (countWork)
    {
      sleep_enable();
      sleep_cpu();
    }
    else
    {
      DDRB |= (1 << pinSwitch);
      PORTB |= (1 << pinSwitch);
      _delay_ms(50);
      DS18B20_init();            // Инициализирую DS18B20
      DS18B20_write(0xCC);       // Пропускаю проверку серийного номера DS18B20
      DS18B20_write(0x44);       // Запускаю температурное преобразование
      _delay_ms(750);            // Жду окончания температурного преобразования
      DS18B20_init();            // Инициализирую DS18B20
      DS18B20_write(0xCC);       // Пропускаю проверку серийного номера DS18B20
      DS18B20_write(0xBE);       // Команда на чтение содержимого ОЗУ
      DS18B20_read_scratchpad(); // Считываю "блокнот"

      Temperature = ((DS_scratchpad[1] << 8) | DS_scratchpad[0]) / 16.0;
      // Temperature = (int8_t)((DS_scratchpad[1] << 4) | (DS_scratchpad[0] >> 4));

      char res[2];
      ftoa(Temperature, res, 2);
      str_uart_send(res);
      _delay_ms(50);
      PORTB &= ~(1 << pinSwitch);
      DDRB &= ~(1 << pinSwitch);
      countWork = count;
      sleep_enable(); // разрешаем сон
      sleep_cpu();    // спать!
    }
  }
}