#include "BNO055.h"


// set up registers/vals
#define CHIP_ID_ADDR        0x00
#define CHIP_ID_VALUE       0xA0

#define PAGE_ID_ADDR        0x07
#define OPR_MODE_ADDR       0x3D
#define PWR_MODE_ADDR       0x3E
#define UNIT_SEL_ADDR       0x3B

#define OPR_MODE_CONFIG     0x00
#define OPR_MODE_NDOF       0x0C   // 9DOF fusion mode = 0x1100 = C

#define PWR_MODE_NORMAL     0x00

// output data registers/vals
// registers are order sequentially for all axis and LSB/MSB so only need 1st address
#define ACC_DATA_X_LSB      0x08
#define GYR_DATA_X_LSB      0x14
#define EUL_DATA_X_LSB      0x1A


/*
We added these two functions since they're definitely beyond what we expect from you as recruits
I'd be impressed if you understood the math behind them.
Basically just know that quats are 4 axis (1 real, 3 imaginary) numbers that allow us 
to nicely talk about rotation, and the functions spit out pitch roll and yaw. 
*/
void BNO055::get_quaternion(BNO055_QUATERNION_TypeDef *result)
{
	if (cantReadDataCount > 0 && cantReadDataCount < 50) {
		cantReadDataCount++;
		return;
	} else if (cantReadDataCount >= 50) {
		cantReadDataCount = 1;
	}
	int16_t w,x,y,z;

	dt[0] = BNO055_QUATERNION_W_LSB;
	int writeResult = _i2c.write(chip_addr, dt, 1, true);
	if (!writeResult)  {
		if (cantReadDataCount > 0) {
			printf("RESET IMU\n");
			reset();
			cantReadDataCount = 0;
		}
		_i2c.read(chip_addr, dt, 8, false);
		w = dt[1] << 8 | dt[0];
		x = dt[3] << 8 | dt[2];
		y = dt[5] << 8 | dt[4];
		z = dt[7] << 8 | dt[6];

		result->w = (double)w / 16384.0f;
		result->x = (double)x / 16384.0f;
		result->y = (double)y / 16384.0f;
		result->z = (double)z / 16384.0f;
	} else {
		cantReadDataCount++;
	}
}

void BNO055::get_angular_position_quat(IMU::EulerAngles *result){

	BNO055_QUATERNION_TypeDef q;
	get_quaternion(&q);

	float roll  = atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)) * 180 / PI;
	float pitch = asin(2 * q.w * q.y - q.x * q.z) * 180 / PI;
	float yaw   = atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)) * 180 / PI;

	memcpy(&result->roll, &roll, sizeof(float));
	memcpy(&result->pitch, &pitch, sizeof(float));
	memcpy(&result->yaw, &yaw, sizeof(float));
}


BNO055::BNO055(I2C &i2c, uint8_t addr) noexcept
: 	_i2c_p(&i2c),
	_i2c(*_i2c_p),
	_res(NC),
	chip_addr(addr),
	cantReadDataCount(0)
{
	this->_addr_write = addr & ~1; // lsb = 0
	this->_addr_read = addr | 1;   // lsb = 1

	_i2c.frequency(100000);

	_res = 1;

	memset(dt, 0, sizeof(dt));
	imuAngles.yaw   = 0.0;
	imuAngles.roll  = 0.0;
	imuAngles.pitch = 0.0;
}

BNO055::BNO055(I2C &i2c, uint8_t addr, PinName p_reset) noexcept
: 	_i2c_p(&i2c),
	_i2c(*_i2c_p),
	_res(p_reset),
	chip_addr(addr),
	cantReadDataCount(0)
{
	this->_addr_write = addr & ~1; // lsb = 0
	this->_addr_read = addr | 1;   // lsb = 1

	_i2c.frequency(100000);

	_res = 1;

	memset(dt, 0, sizeof(dt));
	imuAngles.yaw   = 0.0;
	imuAngles.roll  = 0.0;
	imuAngles.pitch = 0.0;
}

void BNO055::init() noexcept {
	// valid chip addr and chip val
	dt[0] = CHIP_ID_ADDR;
	_i2c.write(_addr_write, dt, 1, false);
	_i2c.read(_addr_read, dt, 1, true);

	if ((uint8_t)dt[0] != CHIP_ID_VALUE) {
		return;
	}

	// set page id -> 0
	dt[0] = PAGE_ID_ADDR;
	dt[1] = 0x00;
	_i2c.write(_addr_write, dt, 2, false);

	// switch to config mode
	// technically bno already in config mode during power on or reset, but this forces it anyways in case it wasn't
	dt[0] = OPR_MODE_ADDR;
	dt[1] = OPR_MODE_CONFIG;
	_i2c.write(_addr_write, dt, 2, false);
	ThisThread::sleep_for(20ms);

	// force reset chip = default power on state
	reset();

	// same as above, verify chip id and val
	dt[0] = CHIP_ID_ADDR;
	_i2c.write(_addr_write, dt, 1, false);
	_i2c.read(_addr_read, dt, 1, true);

	if ((uint8_t)dt[0] != CHIP_ID_VALUE) {
		return;
	}

	// set normal power mode
	dt[0] = PWR_MODE_ADDR;
	dt[1] = PWR_MODE_NORMAL;
	_i2c.write(_addr_write, dt, 2, false);
	ThisThread::sleep_for(20ms);

	// set page id -> 0
	dt[0] = PAGE_ID_ADDR;
	dt[1] = 0x00;
	_i2c.write(_addr_write, dt, 2, false);

	// configure output units
	dt[0] = UNIT_SEL_ADDR;
	dt[1] = 0x00;
	_i2c.write(_addr_write, dt, 2, false);
	ThisThread::sleep_for(20ms);

	// change from config to fusion mode
	change_fusion_mode(OPR_MODE_NDOF);
	ThisThread::sleep_for(20ms);

	// reset imu vals
	imuAngles.yaw   = 0.0;
	imuAngles.roll  = 0.0;
	imuAngles.pitch = 0.0;
}

void BNO055::reset() noexcept {
	_res = 0;
	ThisThread::sleep_for(1ms);
	_res = 1;
	ThisThread::sleep_for(700ms);
}

void BNO055::get_accel(BNO055_VECTOR_TypeDef *la) {
	dt[0] = ACC_DATA_X_LSB;

	_i2c.write(_addr_write, dt, 1, false);
	_i2c.read(_addr_read, dt, 6, true);

	int16_t x = (int16_t)((uint16_t)dt[1] << 8 | dt[0]);
	int16_t y = (int16_t)((uint16_t)dt[3] << 8 | dt[2]);
	int16_t z = (int16_t)((uint16_t)dt[5] << 8 | dt[4]);

	la->x = x / 100.0;
	la->y = y / 100.0;
	la->z = z / 100.0;
}

void BNO055::get_gyro(BNO055_VECTOR_TypeDef *gr) {
  	dt[0] = GYR_DATA_X_LSB;

	_i2c.write(_addr_write, dt, 1, false);
	_i2c.read(_addr_read, dt, 6, true);

	int16_t x = (int16_t)((uint16_t)dt[1] << 8 | dt[0]);
	int16_t y = (int16_t)((uint16_t)dt[3] << 8 | dt[2]);
	int16_t z = (int16_t)((uint16_t)dt[5] << 8 | dt[4]);

	gr->x = x / 16.0;
	gr->y = y / 16.0;
	gr->z = z / 16.0;
}

void BNO055::change_fusion_mode(uint8_t mode) {
	dt[0] = OPR_MODE_ADDR;
	dt[1] = mode & 0x0F; // only set least sig 4 bits
	_i2c.write(_addr_write, dt, 2, false);

	ThisThread::sleep_for(20ms);
}

IMU::EulerAngles BNO055::read() {
	dt[0] = EUL_DATA_X_LSB;

	_i2c.write(_addr_write, dt, 1, false);
	_i2c.read(_addr_read, dt, 6, true);

	int16_t yaw = (int16_t)((uint16_t)dt[1] << 8 | dt[0]);
	int16_t roll = (int16_t)((uint16_t)dt[3] << 8 | dt[2]);
	int16_t pitch = (int16_t)((uint16_t)dt[5] << 8 | dt[4]);

	this->imuAngles.yaw = yaw / 16.0;
	this->imuAngles.roll = roll / 16.0;
	this->imuAngles.pitch = pitch / 16.0;

	return this->imuAngles;
}

IMU::EulerAngles BNO055::getImuAngles() {
  	return this->imuAngles;
}