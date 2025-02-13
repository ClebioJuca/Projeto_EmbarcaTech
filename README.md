# Sistema Preditivo de Falhas em Máquinas Industriais

## Descrição
Este projeto propõe um sistema de monitoramento remoto para máquinas industriais, utilizando sensores para coletar dados e um protocolo de comunicação para enviá-los a um banco de dados. O objetivo é prever falhas e melhorar a segurança e eficiência das máquinas.

## Requisitos
- **Hardware:**
  - Raspberry Pi Pico W (RP2040)
  - Sensor de temperatura NTC
  - Sensor de vibração MPU6050
  - Display OLED SSD1306
  - Resistor de 10kΩ

- **Software:**
  - Raspberry Pi Pico SDK
  - Protocolo MQTT
  - CMake
  - GCC ARM

## Instalação
1. Clone este repositório:
   ```sh
   git clone https://github.com/ClebioJuca/Projeto_EmbarcaTech
   ```
2. Acesse a pasta do projeto:
   ```sh
   cd Projeto_EmbarcaTech
   ```
3. Compile o firmware:
   ```sh
   mkdir build && cd build
   cmake ..
   make
   ```
4. Envie o arquivo .uf2 para o Raspberry Pi Pico W.

## Funcionamento
- Os sensores coletam dados de temperatura e vibração.
- Os dados são enviados via MQTT para um broker.
- O display OLED exibe os valores em tempo real.
- O sistema pode receber comandos via MQTT, como "start" e "stop".

## Testes e Validação
- Foi utilizado o simulador **Wokwi** e a placa **BitDogLab** para testes.
- Testes de comunicação MQTT realizados com **HiveMQ Cloud**.

## Licença
Este projeto está licenciado sob a **GPLv3**. Consulte o arquivo LICENSE para mais informações.

## Contato
Caso tenha dúvidas ou sugestões, entre em contato pelo [GitHub](https://github.com/ClebioJuca/Projeto_EmbarcaTech).

