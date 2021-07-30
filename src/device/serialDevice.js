/* Copyright (c) 2015 - 2018, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Use in source and binary forms, redistribution in binary form only, with
 * or without modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 2. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 3. This software, with or without modification, must only be used with a Nordic
 *    Semiconductor ASA integrated circuit.
 *
 * 4. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/* eslint-disable */
import { fork } from 'child_process';
import path from 'path';
import { getAppDir, logger } from 'pc-nrfconnect-shared';
//import { bilinearInterpolation } from "simple-bilinear-interpolation";
import PPKCmd from '../constants';
import Device, { convertFloatToByteBuffer } from './abstractDevice';
import interpolateArray from '2d-bicubic-interpolate';
import { waitForDebugger } from 'inspector';

/* eslint-disable no-bitwise */
/* eslint-disable */
const generateMask = (bits, pos) => ({ pos, mask: (2 ** bits - 1) << pos });
const MEAS_ADC = generateMask(14, 0);
const MEAS_RANGE = generateMask(3, 14);
const MEAS_COUNTER = generateMask(6, 18);
const MEAS_LOGIC = generateMask(8, 24);

const MAX_PAYLOAD_COUNTER = 0b111111; // 0x3f, 64 - 1
const DATALOSS_THRESHOLD = 500; // 500 * 10us = 5ms: allowed loss
const CALIBRATION_INTERPOLATION_DEGREE = 5; //The amout of points placed inbetween each calibration data point

const getMaskedValue = (value, { mask, pos }) => (value & mask) >> pos;

function getDiaDist(point) {
	return Math.sqrt(Math.pow(point.x, 2) + Math.pow(point.y, 2))
}

function getDistance(p1, p2) {
	return getDiaDist({ x: p1.x - p2.x, y: p1.y - p2.y })
}

function getNearestPoint(arr, point) {
	let min = Infinity;
	let result = arr[0];
	for (let i = 0; i < arr.length; i++) {
		let dist = getDistance(arr[i], point);
		if (dist < min) {
			min = dist
			result = arr[i];
		}
	}
	return result;
}


class SerialDevice extends Device {
	adcMult = 1.8 / 16384;

	//Voltage, Reference, PPK2 Raw Out
	//Calibration Data has to be in a grid for the underlying interpolation library

	//Range 0 0nA-10nA-3uA-5uA
	range0CalData = [
		{ x: 0.8, y: 10.0e-9,  z: 99.50e-9 },
		{ x: 3.3, y: 10.0e-9,  z: 129.5e-9 },
		{ x: 5.0, y: 10.0e-9,  z: 121.5e-9 },
		{ x: 0.8, y: 220.0e-9, z: 301.5e-9 },
		{ x: 3.3, y: 220.0e-9, z: 331.2e-9 },
		{ x: 5.0, y: 220.0e-9, z: 322.7e-9 },
		{ x: 0.8, y: 5.0e-6,   z: 5.086e-6 },
		{ x: 3.3, y: 5.0e-6,   z: 5.112e-6 },
		{ x: 5.0, y: 5.0e-6,   z: 5.113e-6 }];

	//Range 1 (470Ohm) 4uA-6uA-100uA-120uA
	range1CalData = [
		{ x: 0.8, y: 4.0e-6,   z: 6.495e-6 },
		{ x: 3.3, y: 4.0e-6,   z: 6.145e-6 },
		{ x: 5.0, y: 4.0e-6,   z: 5.835e-6 },
		{ x: 0.8, y: 22.0e-6,  z: 23.65e-6 },
		{ x: 3.3, y: 22.0e-6,  z: 23.27e-6 },
		{ x: 5.0, y: 22.0e-6,  z: 22.98e-6 },
		{ x: 0.8, y: 120.0e-6, z: 117.4e-6 },
		{ x: 3.3, y: 120.0e-6, z: 116.9e-6 },
		{ x: 5.0, y: 120.0e-6, z: 116.7e-6 }];

	//Range 2 (22Ohm) 110uA-130uA-1mA-2mA
	range2CalData = [
		{ x: 0.8, y: 110.0e-6, z: 192.9e-6 },
		{ x: 3.3, y: 110.0e-6, z: 155.0e-6 },
		{ x: 5.0, y: 110.0e-6, z: 147.3e-6 },
		{ x: 0.8, y: 470.0e-6, z: 505.4e-6 },
		{ x: 3.3, y: 470.0e-6, z: 497.0e-6 },
		{ x: 5.0, y: 470.0e-6, z: 489.5e-6 },
		{ x: 0.8, y: 2000e-6,  z: 1965e-6 },
		{ x: 3.3, y: 2000e-6,  z: 1957e-6 },
		{ x: 5.0, y: 2000e-6,  z: 1949e-6 }];

	//Range 3 (1Ohm) 2mA-3mA-20mA-50mA
	range3CalData = [
		{ x: 0.8, y: 3.0e-3, z: 4.150e-3 },
		{ x: 3.3, y: 3.0e-3, z: 3.980e-3 },
		{ x: 5.0, y: 3.0e-3, z: 3.805e-3 },
		{ x: 0.8, y: 11.0e-3, z: 11.89e-3 },
		{ x: 3.3, y: 11.0e-3, z: 11.65e-3 },
		{ x: 5.0, y: 11.0e-3, z: 11.51e-3 },
		{ x: 0.8, y: 50.0e-3, z: 49.44e-3 },
		{ x: 3.3, y: 50.0e-3, z: 49.17e-3 },
		{ x: 5.0, y: 50.0e-3, z: 49.98e-3 }];

	//Range 4 (50mOhm) 30mA-60mA-1A-1A
	range4CalData = [
		{ x: 0.8, y: 30.0e-3, z: 59.98e-3 },
		{ x: 3.3, y: 30.0e-3, z: 54.76e-3 },
		{ x: 5.0, y: 30.0e-3, z: 52.37e-3 },
		{ x: 0.8, y: 200.0e-3, z: 256.9e-3 },
		{ x: 3.3, y: 200.0e-3, z: 245.1e-3 },
		{ x: 5.0, y: 200.0e-3, z: 239.9e-3 },
		{ x: 0.8, y: 1000.0e-3, z: 1192e-3 },
		{ x: 3.3, y: 1000.0e-3, z: 1147e-3 },
		{ x: 5.0, y: 1000.0e-3, z: 1139e-3 }];

	//Used for old calibration model and definition of the shunt resistors
	modifiers = {
		r: [10000.0, 470.0, 22.0, 1.0, 0.05],		//Shunt Resistors
		gs: [1, 1, 1, 1, 1],						//Not used
		gi: [1.00250626, 1.109215017, 1, 1, 1],		//Gain over input Current
		o: [126e-9, 1050e-9, 0, 0, 0],				//Offset over input Current
		s: [-5.24e-9, 0, 0, 0, 0],					//Gain over input Voltage
		i: [36.19e-9, 0, 0, 0, 0],					//Offset over input voltage
		ug: [1, 1, 1, 1, 1],						//User Gain [CRTL+SHFT+ALT+A] for advanced settings
	};

	/*
	modifiers = {
		r: [1031.64, 101.65, 10.15, 0.94, 0.043],
		gs: [1, 1, 1, 1, 1],
		gi: [1, 1, 1, 1, 1],
		o: [0, 0, 0, 0, 0],
		s: [0, 0, 0, 0, 0],
		i: [0, 0, 0, 0, 0],
		ug: [1, 1, 1, 1, 1],
	};
	*/

	/*
	modifiers = {
		r: [10000.0, 110.0, 110.0, 10.0, 0.51],
		gs: [1, 111.3335, 17.3798, 2.427, 0.071],
		gi: [1, 0.9692, 0.9649, 0.9543, 0.961],
		o: [115.6761, 74.1289, 62.7129, 48.6302, 85.9166],
		s: [5.5e-8, 5.65e-7, 0.000005916, 0.000066286, 0.00294132],
		i: [-9.2e-8, -0.000001622, 0.000037591, -0.000399745, -0.009211018],
		ug: [1, 1, 1, 1, 1],
	};
	*/

	calibrationDataSet = [];

	adcSamplingTimeUs = 10;

	resistors = { hi: 1.8, mid: 28, lo: 500 };

	vddRange = { min: 800, max: 5000 };

	triggerWindowRange = { min: 1, max: 100 };

	isRunningInitially = false;

	constructor(deviceInfo) {
		super();

		this.capabilities.maxContinuousSamplingTimeUs = this.adcSamplingTimeUs;
		this.capabilities.samplingTimeUs = this.adcSamplingTimeUs;
		this.capabilities.digitalChannels = true;
		this.capabilities.prePostTriggering = true;
		this.spikeFilter = {
			alpha: 0.18,
			alpha5: 0.06,
			samples: 3,
		};

		//Interpolate (bilinear) the calibration Data in order to provide more data points for offset correction
		this.calibrationDataSet[0] = interpolateArray(this.range0CalData, CALIBRATION_INTERPOLATION_DEGREE);
		this.calibrationDataSet[1] = interpolateArray(this.range1CalData, CALIBRATION_INTERPOLATION_DEGREE);
		this.calibrationDataSet[2] = interpolateArray(this.range2CalData, CALIBRATION_INTERPOLATION_DEGREE);
		this.calibrationDataSet[3] = interpolateArray(this.range3CalData, CALIBRATION_INTERPOLATION_DEGREE);
		this.calibrationDataSet[4] = interpolateArray(this.range4CalData, CALIBRATION_INTERPOLATION_DEGREE);

		//Convert from calibration data to offset correction data (Reference-Measured Current)
		for (let j = 0; j < this.calibrationDataSet.length; j++) {
			for (let i = 0; i < this.calibrationDataSet[j].length; i++) {
				let tmp = this.calibrationDataSet[j][i].y;
				this.calibrationDataSet[j][i].y = this.calibrationDataSet[j][i].z;
				this.calibrationDataSet[j][i].z = tmp - this.calibrationDataSet[j][i].z;
			}
		}

		//Log the interpolated calibration data to the console
		console.log(this.calibrationDataSet);

		this.path = deviceInfo.serialport.path;
		this.child = fork(
			path.resolve(getAppDir(), 'worker', 'serialDevice.js')
		);
		this.parser = null;
		this.resetDataLossCounter();

		this.child.on('message', m => {
			if (!this.parser) {
				console.error('Program logic error, parser is not set.');
				return;
			}
			if (m.data) {
				this.parser(Buffer.from(m.data));
				return;
			}
			console.log(`message: ${JSON.stringify(m)}`);
		});
		this.child.on('close', code => {
			if (code) {
				console.log(`Child process exited with code ${code}`);
			} else {
				console.log('Child process cleanly exited');
			}
		});
	}

	resetDataLossCounter() {
		this.expectedCounter = null;
		this.dataLossCounter = 0;
		this.corruptedSamples = [];
	}

	getAdcResult(range, adcVal) {
		/*
		const resultWithoutGain =
			(adcVal - this.modifiers.o[range]) *
			(this.adcMult / (this.modifiers.r[range]));
		let adc =
			this.modifiers.ug[range] *
			(resultWithoutGain *
				(this.modifiers.gs[range] * resultWithoutGain +
					this.modifiers.gi[range]) +
				(this.modifiers.s[range] * (this.currentVdd / 1000) +
					this.modifiers.i[range]));
		*/

		//Calculate Voltage at ADC Input from Reference Voltage and ADC Counts and multiply by 2 (Gain Correction from Firmware)
		let vADC = (2.0 * adcVal * this.adcMult);
		//Calculate Current from Voltage at the ADC (times 2 bc. of 1:1 voltage divider) div. by 40 (InstAmp Gain) and divide by shunt Res.
		let iADC = ((2.0 * vADC) / 40.0) / (this.modifiers.r[range]);

		//Find the nearest Point in the interpolated calibration Matrix
		let offset = getNearestPoint(this.calibrationDataSet[range], { x: (this.currentVdd / 1000), y: iADC });
		//Apply the offset
		let adc = iADC + offset.z;


		//Gain and Offset Correction over input current
		//let iAdcCorr = (iADC - this.modifiers.o[range]) * this.modifiers.gi[range];
		//Gain and Offset Correction over input voltage
		//let adc = iAdcCorr + ((this.currentVdd / 1000) * this.modifiers.s[range] + this.modifiers.i[range]);

		//This kind of calibration is not referring to any of the original calibration models and utilizes a 9 point calibration method across the entire
		//current and voltage range

		//Only update Debugging Messages every n samples in order to prevent logger overload and improve performance
		if (this.debugCounter >= this.debugTrigger) {
			logger.info("Raw-ADC:", adcVal);
			logger.info("I-ADC:", iADC);
			logger.info("Cal-Ofset:", offset.z);
			logger.info("ADC-Fin:", adc);
			logger.info("R-Val:", (this.modifiers.r[range]));
			logger.info("Vdd:", (this.currentVdd / 1000));
			if (this.prevRange != range) {
				//Report as error to display as RED in console
				logger.error('Switched Range:', range, (this.modifiers.r[range]))
			}
			/*
			logger.info("ug:", (this.modifiers.ug[range]));
			logger.info("o:", (this.modifiers.o[range]));
			logger.info("gs:", (this.modifiers.gs[range]));
			logger.info("gi:", (this.modifiers.gi[range]));
			logger.info("s:", (this.modifiers.s[range]));
			logger.info("i:", (this.modifiers.i[range]));
			*/
			this.debugCounter = 0;
		}
		else {
			this.debugCounter++;
		}

		const prevRollingAvg4 = this.rollingAvg4;
		const prevRollingAvg = this.rollingAvg;

		this.rollingAvg =
			this.rollingAvg === undefined
				? adc
				: this.spikeFilter.alpha * adc +
				(1.0 - this.spikeFilter.alpha) * this.rollingAvg;
		this.rollingAvg4 =
			this.rollingAvg4 === undefined
				? adc
				: this.spikeFilter.alpha5 * adc +
				(1.0 - this.spikeFilter.alpha5) * this.rollingAvg4;

		if (this.prevRange === undefined) {
			this.prevRange = range;
		}

		if (this.prevRange !== range || this.afterSpike > 0) {
			if (this.prevRange !== range) {
				// number of measurements after the spike which still to be averaged
				this.consecutiveRangeSample = 0;
				this.afterSpike = this.spikeFilter.samples;
			} else {
				this.consecutiveRangeSample += 1;
			}
			// Use previous rolling average if within first two samples of range 4
			if (range === 4) {
				if (this.consecutiveRangeSample < 2) {
					this.rollingAvg4 = prevRollingAvg4;
					this.rollingAvg = prevRollingAvg;
				}
				adc = this.rollingAvg4;
			} else {
				adc = this.rollingAvg;
			}
			// adc = range === 4 ? this.rollingAvg4 : this.rollingAvg;
			this.afterSpike -= 1;
		}
		this.prevRange = range;

		return adc;
	}

	start() {
		this.child.send({ open: this.path });
		return this.getMetadata();
	}

	parseMeta(m) {
		/*
		Object.keys(this.modifiers).forEach(k => {
			for (let i = 0; i < 5; i += 1) {
				this.modifiers[k][i] = m[`${k}${i}`] || this.modifiers[k][i];
			}
			logger.info('Cal: ', this.modifiers[k]);
		})
		*/
		return m;
	}

	stop() {
		this.child.kill();
	}

	sendCommand(cmd) {
		if (cmd.constructor !== Array) {
			this.emit(
				'error',
				'Unable to issue command',
				'Command is not an array'
			);
			return undefined;
		}
		if (cmd[0] === PPKCmd.AverageStart) {
			this.rollingAvg = undefined;
			this.rollingAvg4 = undefined;
			this.prevRange = undefined;
			this.consecutiveRangeSample = 0;
			this.afterSpike = 0;
		}
		this.child.send({ write: cmd });
		return Promise.resolve(cmd.length);
	}

	dataLossReport(missingSamples) {
		if (
			this.dataLossCounter < DATALOSS_THRESHOLD &&
			this.dataLossCounter + missingSamples >= DATALOSS_THRESHOLD
		) {
			logger.error(
				'Data loss detected. See https://github.com/Nordicsemiconductor/pc-nrfconnect-ppk/blob/master/doc/troubleshooting.md#data-loss-with-ppk2'
			);
		}
		this.dataLossCounter += missingSamples;
	}

	handleRawDataSet(adcValue) {
		try {
			const currentMeasurementRange = Math.min(
				getMaskedValue(adcValue, MEAS_RANGE),
				this.modifiers.r.length
			);
			const counter = getMaskedValue(adcValue, MEAS_COUNTER);
			const adcResult = getMaskedValue(adcValue, MEAS_ADC) * 4;
			const bits = getMaskedValue(adcValue, MEAS_LOGIC);
			const value =
				this.getAdcResult(currentMeasurementRange, adcResult) * 1e6;

			if (this.expectedCounter === null) {
				this.expectedCounter = counter;
			} else if (
				this.corruptedSamples.length > 0 &&
				counter === this.expectedCounter
			) {
				while (this.corruptedSamples.length > 0) {
					this.onSampleCallback(this.corruptedSamples.shift());
				}
				this.corruptedSamples = [];
			} else if (this.corruptedSamples.length > 4) {
				const missingSamples =
					(counter - this.expectedCounter + MAX_PAYLOAD_COUNTER) &
					MAX_PAYLOAD_COUNTER;
				this.dataLossReport(missingSamples);
				for (let i = 0; i < missingSamples; i += 1) {
					this.onSampleCallback({});
				}
				this.expectedCounter = counter;
				this.corruptedSamples = [];
			} else if (this.expectedCounter !== counter) {
				this.corruptedSamples.push({ value, bits });
			}

			this.expectedCounter += 1;
			this.expectedCounter &= MAX_PAYLOAD_COUNTER;
			// Only fire the event, if the buffer data is valid
			this.onSampleCallback({ value, bits });
		} catch (err) {
			console.log(err.message, 'original value', adcValue);
			// to keep timestamp consistent, undefined must be emitted
			this.onSampleCallback({});
		}
	}

	remainder = Buffer.alloc(0);

	parseMeasurementData(buf) {
		const sampleSize = 4;
		let ofs = this.remainder.length;
		const first = Buffer.concat(
			[this.remainder, buf.subarray(0, sampleSize - ofs)],
			sampleSize
		);
		ofs = sampleSize - ofs;
		this.handleRawDataSet(first.readUIntLE(0, sampleSize));
		for (; ofs <= buf.length - sampleSize; ofs += sampleSize) {
			this.handleRawDataSet(buf.readUIntLE(ofs, sampleSize));
		}
		this.remainder = buf.subarray(ofs);
	}

	getMetadata() {
		let metadata = '';
		return (
			new Promise(resolve => {
				this.parser = data => {
					metadata = `${metadata}${data}`;
					if (metadata.includes('END')) {
						// hopefully we have the complete string, HW is the last line
						this.parser = this.parseMeasurementData.bind(this);
						resolve(metadata);
					}
				};
				this.sendCommand([PPKCmd.GetMetadata]);
			})
				// convert output string json:
				.then(m =>
					m
						.replace('END', '')
						.trim()
						.toLowerCase()
						.replace(/-nan/g, 'null')
						.replace(/\n/g, ',\n"')
						.replace(/: /g, '": ')
				)
				.then(m => `{"${m}}`)
				// resolve with parsed object:
				.then(JSON.parse)
		);
	}

	// Capability methods

	ppkSetPowerMode(isSmuMode) {
		return this.sendCommand([PPKCmd.SetPowerMode, isSmuMode ? 2 : 1]);
	}

	ppkSetUserGains(range, gain) {
		this.modifiers.ug[range] = gain;
		return this.sendCommand([
			PPKCmd.SetUserGains,
			range,
			...convertFloatToByteBuffer(gain),
		]);
	}

	ppkSetSpikeFilter(spikeFilter) {
		this.spikeFilter = {
			...this.spikeFilter,
			...spikeFilter,
		};
	}

	ppkAverageStart() {
		this.resetDataLossCounter();
		return super.ppkAverageStart();
	}

	ppkTriggerSet() {
		return this.ppkAverageStart();
	}

	ppkTriggerStop() {
		return this.ppkAverageStop();
	}

	ppkTriggerSingleSet() {
		return this.ppkAverageStart();
	}
}

export default SerialDevice;
