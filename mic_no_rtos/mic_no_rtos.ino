#include <driver/i2s.h>
#include <SD.h>

#define I2S_WS 25
#define I2S_SD 34
#define I2S_SCK 26

#define I2S_PORT I2S_NUM_0

#define RANGO_MEDICIONES 65300  // Menor valor, menor sensibilidad
#define TIEMPO_DE_GRABACION (5) // En segundos. No modificar por ahora, la logica todavia no se adapta a estos cambios.
#define NOMBRE_ARCHIVO_AUDIO "AUDIO"
#define I2S_SAMPLE_RATE (16000)
#define I2S_SAMPLE_BITS (16)

#define I2S_READ_LEN (16 * 1024)
#define I2S_CHANNEL_NUM (1)
#define FLASH_RECORD_SIZE (I2S_CHANNEL_NUM * I2S_SAMPLE_RATE * I2S_SAMPLE_BITS / 8 * TIEMPO_DE_GRABACION)
#define HEADER_SIZE 44

char filename[50];
int indice = 0;
unsigned long volumen_inicial = 0;
unsigned long volumen_final = 0;
unsigned long volumen_ideal = 0;

bool flag_a = false;
bool flag_b = false;

byte header[HEADER_SIZE];

// SETUP TAREA
int i2s_read_len = I2S_READ_LEN;
int flash_wr_size = 0;
size_t bytes_read;

char *i2s_read_buff = (char *)calloc(i2s_read_len, sizeof(char));
uint8_t *flash_write_buff = (uint8_t *)calloc(i2s_read_len, sizeof(char));

File file;

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);

  inicializar_sd();
  inicializar_i2s();
  inicializar_setup_tarea();

  Serial.println("\n*** ESPERANDO EVENTO ***");
  delay(500);
}

void loop()
{
  // put your main code here, to run repeatedly:
  if (!flag_a)
  {
    file.close();
    volumen_final = SD.usedBytes();
    volumen_ideal = volumen_final - volumen_inicial;

    /*
      if (volumen_ideal < TIEMPO_DE_GRABACION * 36864 && !flag_b)
      {
        Serial.println("El audio anterior no es fantasma :D");
        flag_b = true;
      }
    */

    int32_t sample = 0;
    int bytes = i2s_pop_sample(I2S_PORT, (char *)&sample, portMAX_DELAY);

    if (bytes > 0)
    {
      if (sample > 60000 && sample < RANGO_MEDICIONES && millis() > 5000 && !flag_a)
      {
        Serial.println("\n*** OCURRIO UN EVENTO ***\n");
        Serial.print("Volumen anterior: ");
        Serial.println(volumen_final - volumen_inicial);

        volumen_inicial = SD.usedBytes();

        sprintf(filename, "/%s_%u.wav", NOMBRE_ARCHIVO_AUDIO, indice++);

        SD.remove(filename);
        file = SD.open(filename, FILE_WRITE);

        if (file)
        {
          wavHeader(header, FLASH_RECORD_SIZE);
          file.write(header, HEADER_SIZE);

          ets_printf(" Iniciando: %s\n ", filename);

          flag_a = true;
        }
        else
          Serial.println("Error al abrir archivo");
      }
    }
  }

  if (flash_wr_size < FLASH_RECORD_SIZE && flag_a)
  {
    file.write((const byte *)flash_write_buff, i2s_read_len);

    i2s_read(I2S_PORT, (void *)i2s_read_buff, i2s_read_len, &bytes_read, portMAX_DELAY);
    i2s_adc_data_scale(flash_write_buff, (uint8_t *)i2s_read_buff, i2s_read_len);

    flash_wr_size += i2s_read_len;

    ets_printf("Grabando %u%\n", flash_wr_size * 100 / FLASH_RECORD_SIZE);
  }
  else if (!flash_wr_size < FLASH_RECORD_SIZE && flag_a)
  {
    file.close();
    flag_a = false;
    flag_b = false;
    flash_wr_size = 0;

    Serial.println("Grabacion finalizada!");
  }
}

void i2s_adc_data_scale(uint8_t *d_buff, uint8_t *s_buff, uint32_t len)
{
  uint32_t j = 0;
  uint32_t dac_value = 0;
  for (int i = 0; i < len; i += 2)
  {
    dac_value = ((((uint16_t)(s_buff[i + 1] & 0xf) << 8) | ((s_buff[i + 0]))));
    d_buff[j++] = 0;
    d_buff[j++] = dac_value * 256 / 2048;
  }
}

void wavHeader(byte *header, int wavSize)
{
  header[0] = 'R';
  header[1] = 'I';
  header[2] = 'F';
  header[3] = 'F';
  unsigned int fileSize = wavSize + HEADER_SIZE - 8;
  header[4] = (byte)(fileSize & 0xFF);
  header[5] = (byte)((fileSize >> 8) & 0xFF);
  header[6] = (byte)((fileSize >> 16) & 0xFF);
  header[7] = (byte)((fileSize >> 24) & 0xFF);
  header[8] = 'W';
  header[9] = 'A';
  header[10] = 'V';
  header[11] = 'E';
  header[12] = 'f';
  header[13] = 'm';
  header[14] = 't';
  header[15] = ' ';
  header[16] = 0x10;
  header[17] = 0x00;
  header[18] = 0x00;
  header[19] = 0x00;
  header[20] = 0x01;
  header[21] = 0x00;
  header[22] = 0x01;
  header[23] = 0x00;
  header[24] = 0x80;
  header[25] = 0x3E;
  header[26] = 0x00;
  header[27] = 0x00;
  header[28] = 0x00;
  header[29] = 0x7D;
  header[30] = 0x00;
  header[31] = 0x00;
  header[32] = 0x02;
  header[33] = 0x00;
  header[34] = 0x10;
  header[35] = 0x00;
  header[36] = 'd';
  header[37] = 'a';
  header[38] = 't';
  header[39] = 'a';
  header[40] = (byte)(wavSize & 0xFF);
  header[41] = (byte)((wavSize >> 8) & 0xFF);
  header[42] = (byte)((wavSize >> 16) & 0xFF);
  header[43] = (byte)((wavSize >> 24) & 0xFF);
}

void inicializar_setup_tarea()
{
  i2s_read(I2S_PORT, (void *)i2s_read_buff, i2s_read_len, &bytes_read, portMAX_DELAY);
  i2s_read(I2S_PORT, (void *)i2s_read_buff, i2s_read_len, &bytes_read, portMAX_DELAY);
}

void inicializar_i2s()
{
  i2s_config_t i2s_config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
      .sample_rate = I2S_SAMPLE_RATE,
      .bits_per_sample = i2s_bits_per_sample_t(I2S_SAMPLE_BITS),
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
      .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
      .intr_alloc_flags = 0,
      .dma_buf_count = 64,
      .dma_buf_len = 1024,
      .use_apll = 1};

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);

  const i2s_pin_config_t pin_config = {
      .bck_io_num = I2S_SCK,
      .ws_io_num = I2S_WS,
      .data_out_num = -1,
      .data_in_num = I2S_SD};

  i2s_set_pin(I2S_PORT, &pin_config);
}

void inicializar_sd()
{
  Serial.println("\nIniciando SD...");
  if (!SD.begin(5U))
  {
    while (!SD.begin(5U))
      Serial.println("Error al iniciar SD");
  }
  else
    Serial.println("Inicializacion exitosa");
}
