# MD-5_firm
send below commands via CAN with the ID (std 0x00 (for all), ext 0x01 (for motor driver), ext original id).

all data is little endian

|command (#0)|command name|#1|#2|#3|#4|#5|#6|#7|have return|return #0|return #1|return #2|return #3|return #4|return #5|return #6|return #7|
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
|0|stop|																
1|start																
2|reset																
3|get original id|iid low 8 for return|iid high 3 for return||||||TRUE|LSByte 8bit|2nd 8bit|3rd 8bit|MSByte 5bit				
4|set data id|id low 8bit|id high 3bit|read specify number of data													
5|set pwm|low 8bit|high 8bit														

## LISENCE
<a rel="license" href="http://creativecommons.org/licenses/by/4.0/"><img alt="Creative Commons Licence" style="border-width:0" src="https://i.creativecommons.org/l/by/4.0/88x31.png" /></a><br />This work is licensed under a <a rel="license" href="http://creativecommons.org/licenses/by/4.0/">Creative Commons Attribution 4.0 International License</a>.
<br />この 作品 は <a rel="license" href="http://creativecommons.org/licenses/by/4.0/">クリエイティブ・コモンズ 表示 4.0 国際 ライセンス</a>の下に提供されています。
