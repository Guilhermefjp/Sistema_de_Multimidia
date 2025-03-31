# Sistema_de_Multimidia

Projeto realizado na disciplina de Software Embarcado : Sistema de multimídia para veículos usando ESP32 com integração Bluettoh e aplicativo interativo. Realizado em C, utilizando ESP-IDF e FreeRTOS.

## Funções implementadas no projeto 

<ul>
  <li>Sensor de estacionamento</li>
    <ul>
      <li>Utilização de um sensor ultrassonico HC-SR04 e um buzzer para indicar a proximidade do sensor a um obstáculo.</li>
    </ul>
  <li>Comando de Farol Automático</li>
    <ul>
      <li>Utilizaçã de um fotodiodo LDR para variar a resistência com a baixa luminosidade incidente e a partir de um ADC comandar o acionamento dos faróis.</li>
    </ul>
 <li>Implementação de GPS</li>
    <ul>
      <li>Utilização do Módulo GPS Neo-6M para obter a localização exata do sensor.</li>
    </ul>
 <li>Integração Bluetooth</li>
    <ul>
      <li>Utilização do módulo Bluetooth do ESP no modo low energy para transmitir os dados adquiridos e processados.</li>
    </ul>
  <li>App para vizsualização pelo usuário</li>
    <ul>
      <li>Criado no MTI App Inventor, apresenta a localização em um mapa com base nos valores entregues pelo GPS; comanda o farol ou deixa no modo automático; e mostra o valor medido pelo sensor de distância no modo estacionamento. </li>
    </ul>
</ul>

![image](https://github.com/user-attachments/assets/0a2e050d-b574-4947-965a-358935cb2bf3)
