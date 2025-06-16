ー　工事中　（Under construction）ー

# MEZ68K8_RAM Rev2.1 for FUZIX 0.5

MEZ68K8_RAM Rev2.1は[EMUZ80](https://vintagechips.wordpress.com/2022/03/05/emuz80_reference/)ボード上で動く68008メザニンボードです。



（MEZ68K8_RAM Rev2.1メザニンボード）
![](photo/P1030052.JPG)<br>


EMUZ80はMicrochip社のPIC18F47Q43/83/84を使用して、Z80を制御するシングルボードコンピュータ
です。また、Z80の代わりにメザニンボードを装着することで、レガシーCPU（6502や、8088、V20、他）
を動かすことが出来ます。<br>
UART, SPI, I2C等の制御をPICで行い、レガシーCPUの外部I/Oとして機能を提供します。
SDカードはSPIを通して実装されています。<br>
PICから、レガシーCPUのメモリ空間にアクセスすることで、レガシーPCのプログラムをロードする
ことが出来るため、ROMは必要なくSRAMでプログラムを動作させることが出来ます。


（EMUZ80、MEZ68K8_RAM、RTCモジュール、SDカードモジュール）
![](photo/P1030046.JPG)<br>


MEZ68K8_RAM Rev2.1の[FUZIX](https://www.fuzix.org/)は、platform-[pico68k](https://hackaday.io/project/179200-68000-minimal-homebrew-computer)をベースにポーティングを行いました。


リアルタイムクロックは、platfoem-[68knano](https://github.com/74hc595/68k-nano)のds3234デバイスの実装を参考にしています。

MEZ68K8_RAM Rev2.1では、FUZIX以外に、CP/M-68Kや68000用の[Universal Monitor](https://electrelic.com/electrelic/node/1317)をサポートしていますが、詳細については、[MEZ68K8_RAM(Rev1.3)](https://github.com/akih-san/MEZ68K8_RAM)を参照してください。

## [MEZ68K8_RAM(Rev1.3)](https://github.com/akih-san/MEZ68K8_RAM)からの追加変更点

１．IRQ, NMI割り込みのサポート<br>
２．RTC(DS1307)のサポート<br>

### １．IRQ, NMI割り込みのサポート
Rev1.3では割り込みなく、最小限の機能を実現していました。Rev2.1では、オートベクターによるレベル４(IRQ)とレベル７(NMI)の割り込みをサポートします。
