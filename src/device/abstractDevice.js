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

import EventEmitter from 'events';

import PPKCmd from '../constants';

/* eslint-disable class-methods-use-this */

const getAllPropertyNames = obj => {
    const proto = Object.getPrototypeOf(obj);
    const inherited = proto ? getAllPropertyNames(proto) : [];
    return [...new Set(Object.getOwnPropertyNames(obj).concat(inherited))];
};

export function convertFloatToByteBuffer(floatnum) {
    const float = new Float32Array(1);
    float[0] = floatnum;
    const bytes = new Uint8Array(float.buffer);
    return bytes;
}

export default class Device extends EventEmitter {
    currentVdd = 0;
    prevVdd = 0;
    debugCounter = 0;
    debugTrigger = 100000;

    triggerWindowRange = { min: 1, max: 10 };

    constructor() {
        super();
        this.capabilities = {};
        getAllPropertyNames(this)
            .filter(k => k.startsWith('ppk'))
            .forEach(k => Object.assign(this.capabilities, { [k]: true }));
    }

    async open() {
        throw new Error('not implemented');
    }

    async sendCommand() {
        throw new Error('not implemented');
    }

    async stop() {
        throw new Error('not implemented');
    }

    async start() {
        throw new Error('not implemented');
    }

    onSampleCallback() {
        throw new Error('not implemented');
    }

    parseMeta() {
        throw new Error('not implemented');
    }

    // Capability methods

    ppkAverageStart() {
        return this.sendCommand([PPKCmd.AverageStart]);
    }

    ppkAverageStop() {
        return this.sendCommand([PPKCmd.AverageStop]);
    }

    ppkDeviceRunning(...args) {
        return this.sendCommand([PPKCmd.DeviceRunningSet, ...args]);
    }

    ppkUpdateRegulator(vdd) {
        this.currentVdd = vdd;
        // eslint-disable-next-line no-bitwise
        return this.sendCommand([PPKCmd.RegulatorSet, vdd >> 8, vdd & 0xff]);
    }
}
