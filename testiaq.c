
#include "rm_zmod4xxx.h"
#include "r_i2c_master.h"

rm_zmod4xxx_ctrl_t sensor_ctrl;
rm_zmod4xxx_cfg_t sensor_cfg;
rm_zmod4xxx_raw_data_t raw_data;

// Initialisation I2C
R_I2C_Open(&i2c_ctrl, &i2c_config);

// Initialiser le capteur
p_zmod4xxx_api->open(&sensor_ctrl, &sensor_cfg);

// Demarrer la mesure
p_zmod4xxx_api->measurementStart(&sensor_ctrl);

// Attendre que la mesure soit faite
// (par interruption ou poll de status)

// Lire les données
p_zmod4xxx_api->read(&sensor_ctrl, &raw_data, NULL);

// Utiliser raw_data (IAQ / TVOC)
