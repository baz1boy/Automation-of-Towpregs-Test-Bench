#ifndef   __MLX90615_H
#define   __MLX90615_H


#include <Wire.h>
#include <Arduino.h>


#define   MLX90615_ADDRESS    0x00
#define   MLX90615_ADDR_WRITE   0x00	
#define   MLX90615_ADDR_READ    0x01
#define   MLX90615_RAM    0x00
#define   AMBIENT_TEMP    0x06
#define   OBJECT_TEMP     0x07


class  WaveShare_MLX90615{
 public:
  WaveShare_MLX90615(uint8_t addr = MLX90615_ADDRESS);
  boolean begin();
  double readObjectTemp(void);
  double readAmbientTemp(void);

 private:
  uint8_t   _addr;
  float   readTemp(uint8_t reg);
};

#endif 
