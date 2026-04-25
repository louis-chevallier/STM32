
# MCU 

J'ai le mcu STM32F407ZGT6

- pour spécifier la config du hard et générer le code : STM32CubeMX 
  ouvrir beam02/beam02.ioc
  => generate code


- aller dans beam32/Core/Src
  là est le source cpp
  make start
  ca utilise les outils cli : installés dans /mnt/hd1/tools/stm32ide/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.linux64_1.0.100.202602081740/tools/bin/


  je les ai installés avec l'installeur trouvé ici : https://www.st.com/en/development-tools/stm32cubeide.html
  et https://www.st.com/en/development-tools/stm32cubeprog.html
  prendre la version linux generic ( because linux mint 22 )
  




# connecteur debug

5V ( et pas 3.3)

# clock DAC


[notebook](https://colab.research.google.com/drive/1mBSUoHLU5fF2yGJLQd4LiWSCUpsHnCSH?usp=sharing)


- sys clock freq = 80MHz
- PSC 99
- desired timer freq : 100KHz
- solve : => ARR=9




# pinout

![pinout](pinout.png)

- DAC : PA4 - fil jaune, vers ampli
- ADC : PA6 ( fil bleu), PA7, PB0, PB1

- D1 : fin conversion ADC du groupe , (fil vert)
- B2 : fin conversion DAC   (fil rouge) 
- C13 : led ( freq = k * variance )

- A6, A7, B0, B1 : ADC input

- observation : acquisition a 59.2KHz des 4 canaux
 

# config, freq

| PSC | ARR | cycles | freq KHz |
| --  | --  | --     |  --      |
| 19  | 24  | 15     | 55  |
| 19  | 24  | 28     | 37  |