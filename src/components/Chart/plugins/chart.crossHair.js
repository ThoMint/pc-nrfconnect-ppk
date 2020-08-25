/* Copyright (c) 2015 - 2020, Nordic Semiconductor ASA
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

/* eslint no-param-reassign: off */

const plugin = {
    id: 'crossHair',
    instances: [],
    moveEvent: {},

    pointerMoveHandler(evt, chartInstance) {
        const { layerX, layerY } = evt || {};
        const { chartArea } = chartInstance;
        const { left } = chartArea;
        plugin.moveEvent = { layerX: layerX - left, layerY, id: chartInstance.id };
        plugin.instances.forEach(instance => instance.update({ lazy: true }));
    },

    pointerLeaveHandler() {
        plugin.moveEvent = {};
        plugin.instances.forEach(instance => instance.update({ lazy: true }));
    },

    beforeInit(chartInstance) {
        plugin.instances.push(chartInstance);
        const { canvas } = chartInstance.chart.ctx;
        canvas.addEventListener('pointermove', evt => plugin.pointerMoveHandler(evt, chartInstance));
        canvas.addEventListener('pointerup', evt => plugin.pointerMoveHandler(evt, chartInstance));
        canvas.addEventListener('pointerleave', plugin.pointerLeaveHandler);
    },

    afterDraw(chartInstance) {
        const {
            chartArea, chart, scales,
        } = chartInstance;
        const { ctx } = chart;
        const { canvas } = ctx;
        const {
            left, right, top, bottom,
        } = chartArea;

        if (!plugin.moveEvent) {
            canvas.style.cursor = 'default';
        }

        const { layerX, layerY } = plugin.moveEvent;

        const { xScale, yScale } = scales;
        const [time, subsecond] = this.formatX(xScale.getValueForPixel(left + layerX), 0, []);
        const { width: tsWidth } = ctx.measureText(time);

        ctx.save();
        ctx.lineWidth = 0.5;
        ctx.strokeStyle = 'black';
        ctx.beginPath();
        ctx.moveTo(left + layerX - 0.5, top);
        ctx.lineTo(left + layerX - 0.5, bottom);
        ctx.closePath();
        ctx.stroke();

        if (chartInstance.id === 0) {
            ctx.fillStyle = 'black';
            ctx.textAlign = 'right';
            ctx.fillRect(left + layerX - 5 - (tsWidth / 2), top, tsWidth + 10, 33);
            ctx.fillStyle = 'white';
            ctx.textAlign = 'center';
            ctx.fillText(time, left + layerX, top + 13);
            ctx.fillText(subsecond, left + layerX, top + 28);
        }

        ctx.restore();

        if (!(top < layerY && bottom > layerY && layerX > 0 && right > layerX)) {
            canvas.style.cursor = 'default';
            return;
        }

        canvas.style.cursor = 'pointer';

        if (yScale && plugin.moveEvent.id === 0) {
            ctx.save();
            ctx.lineWidth = 0.5;
            ctx.strokeStyle = 'black';
            ctx.beginPath();
            ctx.moveTo(left, layerY - 0.5);
            ctx.lineTo(right, layerY - 0.5);
            ctx.closePath();
            ctx.stroke();

            const uA = yScale ? this.formatY(yScale.getValueForPixel(layerY)) : null;
            const { width: uAwidth } = ctx.measureText(uA);

            ctx.fillStyle = 'black';
            ctx.textAlign = 'right';
            ctx.fillRect(right - uAwidth - 10, layerY - 10, uAwidth + 10, 20);
            ctx.fillStyle = 'white';
            ctx.fillText(uA, right - 5, layerY + 3);
            ctx.restore();
        }
    },
};

export default plugin;
