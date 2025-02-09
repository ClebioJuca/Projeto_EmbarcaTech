#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"
#include "inc/ssd1306.h"
#include "pico/cyw43_arch.h"
#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "config.h" 

// Comentário para ativar o modo DEBUG, habilitando o DEBUG_PRINT
// #define DEBUG
#ifdef DEBUG
    #define DEBUG_PRINT(fmt, ...) printf(fmt, ##__VA_ARGS__) 
#else
    #define DEBUG_PRINT(fmt, ...)
#endif

//------------------------------------------------------------------------
// Defines para o I2C
#define I2C_SDA0 4
#define I2C_SCL0 5
#define I2C_SDA1 14
#define I2C_SCL1 15

//------------------------------------------------------------------------
// Defines para o Sensor NTC
#define ntc_pin 26                      // Pino gpio 26 (adc0)
#define ntc_temp_channel 0              // Channel do ADC

#define ntc_resistor 10000.0            // Resistor em série (10 kΩ)
#define ntc_nominal_resistance 10000.0  // Resistência nominal do ntc a 25°c
#define ntc_nominal_temperature 25.0    // Temperatura nominal em °c
#define ntc_b_coefficient 3950.0        // Coeficiente beta do ntc // 
#define ntc_average_n 5

// -----------------------------------------------------------------------
// Defines para o Sensor interno de Temperatura do RP2040
#define internal_temp_channel 4
#define internal_average_n 100

// -----------------------------------------------------------------------
// Defines para o Sensor mpu6050
#define mpu6050_addr 0x68 // Endereço I2C do mpu6050

// Registradores do mpu605
#define pwr_mgmt_1   0x6B
#define accel_xout_h 0x3B
#define accel_xout_l 0x3C
#define accel_yout_h 0x3D
#define accel_yout_l 0x3E
#define accel_zout_h 0x3F  
#define accel_zout_l 0x40

// -----------------------------------------------------------------------
// Tensão de referência do adc
float adc_voltage = 3.3f;

volatile bool exit_code = false;

mqtt_client_t *global_mqtt_client = NULL;
volatile absolute_time_t last_mqtt_connect = 0;
// -----------------------------------------------------------------------
 
// Prototipação das Funções utilizadas
float ntc_calculate_temperature();                                                          // Calcula temperatura do NTC

float internal_calculate_temperature();                                                     // Calcula temperatura interna do RP2040

void mpu6050_init(i2c_inst_t *i2c);                                                         // Inicializa o MPU6050    
int16_t read_accel_axis(i2c_inst_t *i2c, uint8_t reg_high, uint8_t reg_low);                // Lê um eixo do MPU6050
float calculate_magnitude(float accel_x, float accel_y, float accel_z);                     // Calcula a magnitude da aceleração

void update_ssd1306(uint8_t ssd[], struct render_area frame_area, float sensor_data[3]);    // Atualiza o display OLED

uint wifi_init();                                                                           // Inicializa o Wi-Fi
void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status); // Callback da conexão MQTT
void mqtt_publish_cb(void *arg, err_t result);                                              // Callback do envio da mensagem
void mqtt_subscribe_cb(void *arg, signed char response_code);                                    // Callback da inscrição no tópico
void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len);                 // Callback da mensagem recebida pelo tópico
void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags);             // Callback dos dados recebidos
void mqtt_start_action(const u8_t *data, u16_t len);                                        // Inicia a ação com base nos dados recebidos                                  // Callback da inscrição no tópico
void start_mqtt_client();                                                                   // Inicializa o cliente MQTT
void reconnect_wifi();                                                                         // Reconecta ao Wi-Fi e ao Broker MQTT  
void display_interruption(uint8_t ssd[], struct render_area frame_area, char* message);     // Exibe a mensagem de interrupção dos sensores

int main() 
{
    stdio_init_all(); 

    adc_init(); 
    adc_set_temp_sensor_enabled(true);
    adc_gpio_init(ntc_pin);

    // Inicialização do i2c0 e i2c1
    i2c_init(i2c0, 400000); 
    gpio_set_function(I2C_SDA0, GPIO_FUNC_I2C); 
    gpio_set_function(I2C_SCL0, GPIO_FUNC_I2C); 
    gpio_pull_up(I2C_SDA0);                     
    gpio_pull_up(I2C_SCL0);
    mpu6050_init(i2c0);

    i2c_init(i2c1, ssd1306_i2c_clock * 1000);
    gpio_set_function(I2C_SDA1, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL1, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA1);
    gpio_pull_up(I2C_SCL1);
    ssd1306_init();

    struct render_area frame_area = 
    {
        .start_column = 0,
        .end_column = ssd1306_width - 1,
        .start_page = 0,
        .end_page = ssd1306_n_pages - 1
    };
    calculate_render_area_buffer_length(&frame_area);

    uint8_t ssd[ssd1306_buffer_length];
    memset(ssd, 0, ssd1306_buffer_length);
    render_on_display(ssd, &frame_area);

    wifi_init();
    start_mqtt_client();

    const char *publish_topic = MQTT_TOPIC_PUBLISH;
    const char *subscribe_topic = MQTT_TOPIC_SUBSCRIBE;
    char payload[25] = "Starting Sensors";

    err_t err_1 = mqtt_subscribe(global_mqtt_client, subscribe_topic, 1, mqtt_subscribe_cb, NULL);
    mqtt_set_inpub_callback(global_mqtt_client, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, NULL);

    if(global_mqtt_client && mqtt_client_is_connected(global_mqtt_client)) 
        {
            // Envia a mensagem de inicialização para o Broker
            err_t err_1 = mqtt_publish(global_mqtt_client, publish_topic, payload, strlen(payload), 1, 0, NULL, NULL);

        }
    
    bool message_sent = true; // Flag para verificar se a mensagem foi enviada

    char *text[] =
    {
        "Ambient Temperature:",
        "RP2040 Temperature:",
        "Magnitude:",
    };
    
    char *suffix[] = 
    {
        " °C",  // Temperatura
        " °C",  // Temperatura (RP2040)
        ""      // Magnitude
    };
    
    bool display_interrupted = false;

    while(true) 
    {
        static absolute_time_t last_update;
        absolute_time_t current_time = get_absolute_time();

        if(absolute_time_diff_us(last_update, current_time) < 500000)
        {
            sleep_ms(10);
            continue;
        }

        if(!exit_code)
        {
            display_interrupted = false;
            last_update = current_time;

            int16_t ax = read_accel_axis(i2c0, accel_xout_h, accel_xout_l);
            int16_t ay = read_accel_axis(i2c0, accel_yout_h, accel_yout_l);
            int16_t az = read_accel_axis(i2c0, accel_zout_h, accel_zout_l);

            // Armazena as leituras dos sensores em um array
            float sensor_data[] = 
            {
                ntc_calculate_temperature(),            // Temperatura do Sensor NTC
                internal_calculate_temperature(),       // Temperatura interna do RP2040
                calculate_magnitude(ax, ay, az)         // Magnitude do Sensor MPU2040
            };

            static bool connected_before = false;
            if(global_mqtt_client && mqtt_client_is_connected(global_mqtt_client) && cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA) > 1) 
            {
                for (uint i = 0; i < 3; i++) 
                {   
                    DEBUG_PRINT("Ambient Temperature %.0f\n", sensor_data[0]);
                    DEBUG_PRINT("RP2040 Temperature %.0f\n", sensor_data[1]);
                    DEBUG_PRINT("Magnitude: %.0f\n", sensor_data[2]);

                    update_ssd1306(ssd, frame_area, sensor_data);
                    // Envia os dados para o Broker
                    sprintf(payload, "%s %.0f %s", text[i], sensor_data[i], suffix[i]);
                    err_t err = mqtt_publish(global_mqtt_client, publish_topic, payload, strlen(payload), 1, 0, NULL, NULL);
                }

                DEBUG_PRINT("Data sent to Broker: %.0f °C, %.0f °C, %.0f\n", sensor_data[0], sensor_data[1], sensor_data[2]);
            }
            else if(connected_before)
                {
                    display_interruption(ssd, frame_area, "Connection Lost");
                    connected_before = false;      
                }
                else
                {
                    reconnect_wifi();
                }
        }
        else if(!display_interrupted)
        {
            display_interruption(ssd, frame_area, "Broker  Command");
            display_interrupted = true;
        }
    }
    return 0;
}

// Função para calcular a temperatura do NTC (Ambiente)
float ntc_calculate_temperature() 
{
    adc_select_input(ntc_temp_channel);
    adc_read();  // Descarta a primeira leitura
    sleep_us(10);
    uint32_t average_value = 0;
    uint16_t adc_value;
  
    for(int i = 0; i < ntc_average_n; i++)
    {
        average_value += adc_read();
    }
    adc_value = average_value/ntc_average_n;

    float voltage = (adc_value / 4095.0f) * adc_voltage;                    // Converte o valor adc em tensão
    float resistance = (ntc_resistor * voltage) / (adc_voltage - voltage);  // Calcula a resistência do ntc / joystick

    // Calcula a temperatura do NTC utilizando a equação de Steinhart-Hart
    float steinhart;
    steinhart = resistance / ntc_nominal_resistance; 
    steinhart = log(steinhart);  
    steinhart /= ntc_b_coefficient;  
    steinhart += 1.0 / (ntc_nominal_temperature + 273.15);  
    steinhart = 1.0 / steinhart; 
    steinhart -= 273.15;  
    return steinhart;
}

//  Função para calcular a Temperatura interna do R2040
float internal_calculate_temperature()
{
    adc_select_input(internal_temp_channel); 
    adc_read();  // Descarta a primeira leitura
    sleep_us(10);                    
    uint16_t adc_value;
    uint32_t average_value = 0;

    for(uint8_t i = 0; i < internal_average_n; i++)
    {
        average_value += adc_read();
    }
    adc_value = average_value/internal_average_n;

    const float conversion_factor = 3.3f / (1 << 12);
    float voltage = (adc_value) * conversion_factor;
    return 27.0f - (voltage - 0.706f) / 0.001721f;
}


float calculate_magnitude(float accel_x, float accel_y, float accel_z) 
{
    // Calculo da magnitude da aceleração de um corpo
    return sqrtf((accel_x * accel_x + accel_y * accel_y + accel_z * accel_z));
}


void mpu6050_init(i2c_inst_t *i2c) 
{
    uint8_t data[2];
    data[0] = pwr_mgmt_1;
    data[1] = 0x00; // Acorda o MPU6050
    i2c_write_blocking(i2c, mpu6050_addr, data, 2, false);
}

int16_t read_accel_axis(i2c_inst_t *i2c, uint8_t reg_high, uint8_t reg_low) 
{
    uint8_t data[2];
    i2c_write_blocking(i2c, mpu6050_addr, &reg_high, 1, true); // Envia o registrador de alto
    i2c_read_blocking(i2c, mpu6050_addr, data, 2, false);      // Lê dois bytes

    // Combinando os dois bytes em um valor de 16 bits
    return (int16_t)((data[0] << 8) | data[1]);
}

void update_ssd1306(uint8_t ssd[], struct render_area frame_area, float sensor_data[3])
{
    char line_1[17];
    char line_2[17];
    char line_3[17];
 
    // Configuração das linhas do display
    if(sensor_data[0] < 0)
        sprintf(line_1, "    %.0f°C     ", sensor_data[0]);
    else
        sprintf(line_1, "     %.0f°C    ", sensor_data[0]);

    sprintf(line_2, "     %.0f°C    ", sensor_data[1]);

    if(sensor_data[2] >= 100)
        sprintf(line_3, "      %.0f     ", sensor_data[2]);
    else
        sprintf(line_3, "       %.0f     ", sensor_data[2]);

    //Matriz 8 x 15
    char *text[] = 
    {
    //  "               "
        "    Ambient    ",
        "  Temperature:  ",
        line_1,
        "   Raspberry   ",
        "  Temperature:  ",
        line_2,
        "   Magnitude:   ",
        line_3,
    };

    int y = 0;
    for (uint i = 0; i < count_of(text); i++)
    {
        ssd1306_draw_string(ssd, 5, y, text[i]);
        y += 8;
    }
    render_on_display(ssd, &frame_area);
}

uint wifi_init()
{
    if (cyw43_arch_init())
    {
        DEBUG_PRINT("Erro ao inicializar o Wi-Fi\n");
        return 1;
    }

    cyw43_arch_enable_sta_mode();
    DEBUG_PRINT("Conectando ao Wi-Fi...\n");

    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 10000))
    {
        DEBUG_PRINT("Falha ao conectar ao Wi-Fi\n");
        return 1;
    }
    else 
        DEBUG_PRINT("Wi-Fi conectado!\n");

    return 0;
}

void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) 
{
    if (status == MQTT_CONNECT_ACCEPTED) 
        DEBUG_PRINT("Conexão MQTT bem-sucedida!\n");
    else 
        DEBUG_PRINT("Falha na conexão MQTT: %d\n", status);
}

void mqtt_subscribe_cb(void *arg, signed char response_code) 
{
    if (response_code == 0) 
        DEBUG_PRINT("Inscrição bem-sucedida no tópico!\n");
    else 
        DEBUG_PRINT("Falha ao se inscrever no tópico. Código de erro: %d\n", response_code);
}

void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len) 
{
    DEBUG_PRINT("Mensagem recebida no tópico '%s', tamanho total: %u\n", topic, tot_len);
}

void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags) 
{
    DEBUG_PRINT("Dados recebidos: %.*s\n", len, (const char *)data);
    mqtt_start_action(data, len);
}

void mqtt_start_action(const u8_t *data, u16_t len)
{
    const char *command[] =  {"stop", "start"};
    bool match[2] = {true, true};

    // Comparação manual byte por byte
    for (int i = 0; i < len; i++) 
    {
        if (data[i] != command[0][i])
            match[0] = false;

        if (data[i] != command[1][i])
            match[1] = false;

        if (!match[0] && !match[1])
            break;
    }

    if ((match[0] && len == 4 && !exit_code)) 
    {
        DEBUG_PRINT("Parando a execução do programa\n");
        exit_code = true;
    } 
    else if ((match[1] && len == 5 && exit_code)) 
    {
        DEBUG_PRINT("Iniciando a execução do programa\n");
        exit_code = false;
    } 
    else
        DEBUG_PRINT("Nada ocorreu\n");
}

void reconnect_wifi()
{
    if(cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA) <= 0)
    {
        DEBUG_PRINT("Tentando reconectar ao Wi-Fi...\n");

        // Tenta reconectar ao Wi-Fi
        if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 10000)) 
        {
            DEBUG_PRINT("Falha ao reconectar ao Wi-Fi\n");
            return;
        } 
        else 
            DEBUG_PRINT("Wi-Fi reconectado!\n");
    }
}

void start_mqtt_client(void) 
{
    global_mqtt_client = mqtt_client_new();
    if (!global_mqtt_client) 
    {
        DEBUG_PRINT("Falha ao criar cliente MQTT\n");
        return;
    }

    ip_addr_t broker_ip;
    IP4_ADDR(&broker_ip, 3 , 78, 25, 67);

    struct mqtt_connect_client_info_t client_info = 
    {
        .client_id = CLIENT_ID,
        .client_user = CLIENT_USER,
        .client_pass = CLIENT_PASS,
        .keep_alive = 60,
    };

    mqtt_client_connect(global_mqtt_client, &broker_ip, 1883, mqtt_connection_cb, NULL, &client_info);
}

void display_interruption(uint8_t ssd[], struct render_area frame_area, char *message)
{
    memset(ssd, 0, ssd1306_buffer_length);
    render_on_display(ssd, &frame_area);

    char *text[] = 
    {
    //  "               "
        "    Sistema    ",
        " Interrompido: ",
        "               ",
        message
    };

    int y = 0;
    for (uint i = 0; i < count_of(text); i++)
    {
        ssd1306_draw_string(ssd, 5, y, text[i]);
        y += 8;
    }
    render_on_display(ssd, &frame_area);
}