# VRM Trifásico Tolerante a Falhas com Controle PI Digital em ESP32

Repositório oficial dos códigos-fonte desenvolvidos para o Trabalho de Conclusão de Curso (TCC) do curso de **Engenharia de Controle e Automação** da **Universidade Federal de Itajubá (UNIFEI) – Campus Itabira**.

## Informações do Trabalho

**Autor:** Túlio Henrique Rozenthal Marques

**Orientador:** Prof. Dr. Luiz Felipe Pugliese

---

## Descrição do Projeto

Este trabalho apresenta o desenvolvimento, modelagem e validação experimental de um **Módulo Regulador de Tensão (VRM - Voltage Regulator Module)** trifásico baseado na topologia de conversores **Buck síncronos intercalados**, realizando a conversão de **12 V para 5 V**.

A principal contribuição do projeto consiste na implementação simultânea de:

- Controle digital de tensão por controlador PI;
- Balanceamento de corrente entre fases;
- Detecção automática de falhas;
- Reconfiguração dinâmica do sistema após falha;
- Supervisão e aquisição de dados em tempo real.

Toda a estratégia de controle foi executada em um único microcontrolador **ESP32**, permitindo uma solução de baixo custo e elevada robustez operacional.

Quando uma falha é detectada em uma das fases de potência, o sistema é capaz de:

1. Identificar a condição anormal;
2. Isolar automaticamente a fase defeituosa;
3. Reconfigurar o interleaving do PWM;
4. Atualizar os ganhos do controlador PI através de *Gain Scheduling*;
5. Manter o fornecimento de energia à carga sem interrupção perceptível.

---

## Arquitetura do Sistema

### Controle Digital

- Controlador PI discreto;
- Frequência de controle: **10 kHz**;
- Discretização pelo método de Tustin;
- Estratégia de *Gain Scheduling* para diferentes condições de operação.

### Gerenciamento de Falhas

- Detecção de sobrecorrente;
- Detecção de corrente reversa;
- Detecção de perda de fase;
- Isolamento automático do canal defeituoso;
- Reconfiguração automática do número de fases ativas.

### Supervisão

- Comunicação serial em alta velocidade;
- Visualização em tempo real das variáveis do sistema;
- Registro de dados experimentais;
- Exportação de logs para análise posterior.

---

## Hardware Utilizado

### Unidade de Controle

- ESP32-WROOM-32
  - Core 1: Malha de controle PI (10 kHz)
  - Core 0: Supervisor, telemetria e gerenciamento de falhas (1 kHz)

### Estágio de Potência

- Topologia Buck síncrona trifásica intercalada;
- Drivers IR2104;
- MOSFETs IRLZ44N;
- Indutores de 100 µH por fase.

### Sensoriamento

- 3 sensores de corrente ACS712-20A (efeito Hall).

### Filtro e Carga

- Banco capacitivo:
  - 3 × 470 µF
  - 100 nF de desacoplamento

- Carga resistiva:
  - 5 × 2,7 Ω em paralelo
  - Resistência equivalente: 0,54 Ω
  - Corrente aproximada: 9,3 A em 5 V

---

## Vídeos e Registros dos Ensaios

As fotos detalhadas da montagem física em bancada e os vídeos demonstrando o funcionamento do protótipo integrado ao supervisório podem ser acessados no link abaixo:

* **Pasta de Mídias (Google Drive):** [https://drive.google.com/drive/folders/1RuFQWf4kRAazAH4eF9_DAFhtoPIbqm_O?usp=sharing]

## Estrutura do Repositório

```text
.
├── vrm_controle.ino
├── validacao_vrm.m
├── dashboard_vrm.py
└── README.md
