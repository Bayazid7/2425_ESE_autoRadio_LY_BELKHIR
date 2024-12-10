# 2425_ESE_autoRadio_LY_BELKHIR
# Compte-Rendu : TP de Synthèse – Autoradio
# Le code se trouve sur la branche master
## 1. Démarrage
1. **Création du projet :**
 
2. **Test de la LED LD2 :**

3. **Test de l’USART2 :**
  
4. **Fonctionnalité `printf`:**
   - `printf` activé via redirection vers l’USART2.
 ```c
int __io_putchar(int chr)
{
	HAL_UART_Transmit(&huart2, (uint8_t*)&chr,1, HAL_MAX_DELAY);
	return chr;
}
 ```

1. **Activation de FreeRTOS (CMSIS V1) :**
   - FreeRTOS configuré et opérationnel.

2. **Fonctionnement du shell :**
   - **Tâche dédiée :** Shell intégré dans une tâche FreeRTOS avec les fonctions :
 ```c 
   void vTaskShell(void * param) ;
   xTaskCreate(vTaskShell,"TaskShell",SHELL_TASK_SIZE,NULL,SHELL_PRIORITY,NULL) ;
 ```
   - **Mode interruption :** Gestion des interruptions configurée.On utilises un semaphores qui est donné dans 
 ```c 
 void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xSemaphoreShell, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
  ```
## 2. Le GPIO Expander et le VU-Mètre

### 2.1 Configuration
1. **Référence du GPIO Expander :**
   - Référence : _MCP23S17-E/SO_.

2. **SPI utilisé : SPI3**
   -  _PC11 => MISO_.
    - _PB5 => MOSI_.
    - _PC10 => SCK_.
    - _PB7 => CS_.

3. **Paramètres STM32CubeIDE :**
   - SPI configuré avec les paramètres standard requis en respectant le fait que le Baude Rate ne depasse pas 10MHz.
   ![Image SPI](/image/spi.png)

### 2.2 Tests
1. **Clignotement LED :**
   - Pour allumer les LED il faut :
     - Ecrire `0x00` dans `IODIRA` et dans `IODIRB` pour mettre en mode `OUTPUT`
     - Ecrie `0` sur les bit de `OLATx` ou 
2. **Chenillard :**
   - Un chenillard a été implémenté dans une tache.
 ```c
      void vTaskChenillard(void *pvParameters) ;
      xTaskCreate(vTaskChenillard, "ChenillardTask", configMINIMAL_STACK_SIZE, NULL, LED_PRIORITY, NULL);
```

### 2.3 Driver
1. **Driver pour LEDs :**
   - Driver implémenté dans `IOExpander.h/IOExpander.c`.
   - Note : Pour faire factionner le composant la commande est la suivantes :
   ![Communication SPI](/image/spi2.png)
2. **Commande shell :**
   - Commandes ajoutées au shell pour allumer/éteindre les LEDs avec la fonction :
 ```c
int led(h_shell_t *h_shell, int argc, char **argv)
{
	if (argc == 3)
	{
		uint8_t pin = (uint8_t)atoi(argv[1]);
		char port = argv[2][0];
		if (pin <= Pin7 && (port == 'A' || port == 'B'))
		{
			if (port == 'A')
			{
				IOExpanderGPIO_WritePin(PortA, pin, 1);
			}
			else
			{
				IOExpanderGPIO_WritePin(PortA, pin, 1);
			}
		}
	}

	return 0;
}
```
## 3. Le CODEC Audio SGTL5000

### 3.1 Configuration préalables
1. **Pins et I2C utilisés :i2c2**
   - _PB10 => SCL_.
   - _PB11 => SDA_.
2. **Configuration SAI2 :**
   - Paramètres configurés pour SAI A en mode master et SAI B en mode slave avec protocole I2S/PCM.
3. **Horloge MCLK :**
   - Activation avec la ligne `__HAL_SAI_ENABLE(&hsai_BlockA2);`.

### 3.2 Configuration I2C
1. **Lecture du registre `CHIP_ID`:**
  ```c
  #define SGTL5000ADRESSE 0x14
  #define SGTL5000REG_ID 0x0000
  uint16_t CHIP_ID = 0 ;
  HAL_I2C_Mem_Read(&hi2c2, SGTL5000ADRESSE, SGTL5000REG_ID, I2C_MEMADD_SIZE_16BIT, CHIP_ID, 1, HAL_MAX_DELAY) ;
  ```
### 3.3 Signaux I2S
1. **Observations oscilloscope :**
### 3.4 Signal audio
1. **Génération :**
2. **Bypass numérique :**
   - Pour le Bypass numérique :
      -  On met  le bit 2 de `CHIP_ANA_HP_CTRL 0x0022` à `1` pour sélectionner le `LINEIN` pour `ADC`.
      -  Et mettre les bits `5:4`  de `CHIP_SSS_CTRL 0x000A` à `00` pour sélectionner `ADC` comme source pour le `DAC`.

## 4. Visualisation
## 5. Filtre RC
1. **Équation :**
   -L'équation différentielle pour un filtre passe-bas RC est donnée par :

$$
V_{out}(t) + RC \frac{dV_{out}(t)}{dt} = V_{in}(t)
$$

2. **Équation de récurrence :**

$$
V_{out}[n] + RC \cdot f_s \cdot (V_{out}[n] - V_{out}[n-1]) = V_{in}[n]
$$

$$
V_{out}[n] \cdot (1 + RC \cdot f_s) = V_{in}[n] + RC \cdot f_s \cdot V_{out}[n-1]
$$

5. **Coefficient calculés (A, B, D) :**
   
$$
A = 1
$$

$$ 
B = RC \cdot f_s 
$$

$$ 
D = \frac{1}{1 + RC \cdot f_s} 
$$ 

7. **Structure `h_RC_filter_t`:**
   ```c
   typedef struct {
       uint32_t coeff_A;
       uint32_t coeff_B;
       uint32_t coeff_D;
       uint16_t out_prev;
   } h_RC_filter_t;

