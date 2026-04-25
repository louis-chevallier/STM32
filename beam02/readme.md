
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