typedef enum {
	MCU_TYPE_1 = 0,
	MCU_TYPE_2,
	MCU_TYPE_3,
	MCU_TYPE_4
} DRONE_MCU_TYPE_e;

typedef enum {
	SERVO_TYPE_1 = 0,
	SERVO_TYPE_2,
	SERVO_TYPE_3,
	SERVO_TYPE_4
} DRONE_SERVO_TYPE_e;

typedef struct {
	DRONE_SERVO_TYPE_e drone_servo_type;
} drone_peripheral_t;

typedef enum {
	DRONE_INIT_OK = 0,
	DRONE_INIT_BAD,
} DRONE_INIT_RESULT_e

typedef struct {
	char ch_name[20];
	DRONE_MCU_TYPE_e drone_mcu_type;
	drone_peripheral_t *drone_peripheral;
} drone_type_t;

DRONE_INIT_RESULT_e drone_init_hardware (drone_type_t *drone_type);
DRONE_INIT_RESULT_e drone_init_core (DRONE_MCU_TYPE_e drone_mcu_type);
DRONE_INIT_RESULT_e drone_init_peripheral (DRONE_PERIPHERAL_t *drone_mcu_type);

void main (void)
{
	static drone_type_t drone_type = {
		. = "Base";
		. = MCU_TYPE_1;
		. = {SERVO_TYPE_1}
	};

	drone_init_hardware(&drone_type);
}

DRONE_INIT_RESULT_e drone_init_hardware (drone_type_t *drone_type)
{

}