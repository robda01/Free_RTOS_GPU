/*
 * LiquidCrystal_I2C.h
 *
 *  Created on: 11/04/2020
 *      Author: Darren
 */

#ifndef LIQUIDCRYSTAL_I2C_H_
#define LIQUIDCRYSTAL_I2C_H_

namespace std {

class LiquidCrystal_I2C {
public:
	virtual ~LiquidCrystal_I2C();
	LiquidCrystal_I2C();
	LiquidCrystal_I2C(const LiquidCrystal_I2C &other);
};

} /* namespace std */

#endif /* LIQUIDCRYSTAL_I2C_H_ */
