//! VN9E30F
//!
//! 6-channel high-side driver.
//!
//! [Datasheet](https://www.st.com/resource/en/datasheet/vn9e30f.pdf)

use crate::{DriverInterface, GlobalStatus};
use embedded_hal::digital::OutputPin;
use embedded_hal_async::{
    delay::DelayNs,
    spi::{SpiBus, SpiDevice},
};
use embedded_hal_bus::spi::DeviceError;

pub struct Driver<'a, BUS, CS, D>
where
    BUS: SpiBus<u8>,
    CS: OutputPin,
    D: DelayNs,
{
    dev: Vn9e30f<DriverInterface<'a, BUS, CS, D>>,
    channels: usize,
}

impl<'a, BUS, CS, D> Driver<'a, BUS, CS, D>
where
    BUS: SpiBus<u8>,
    CS: OutputPin,
    D: DelayNs,
{
    pub fn new(driver: DriverInterface<'a, BUS, CS, D>, channels: usize) -> Self
    where
        BUS: SpiBus<u8>,
        CS: OutputPin,
        D: DelayNs,
    {
        assert!(channels <= 6, "Only up to 6 channel drivers are supported");

        Self {
            dev: Vn9e30f::new(driver),
            channels,
        }
    }

    /// Number of channels that this driver has.
    pub fn channels(&self) -> usize {
        self.channels
    }

    /// Last global status received from the device.
    pub fn global_status(&self) -> GlobalStatus {
        self.dev.interface.last_global_status
    }

    /// Set all control registers to default.
    pub async fn software_reset(&mut self) -> Result<(), DeviceError<BUS, CS>> {
        self.dev.interface.bus.write(&[0xff, 0, 1]).await.unwrap();
        Ok(())
    }

    /// CLear all status registers.
    pub async fn clear_status(&mut self) -> Result<(), DeviceError<BUS, CS>> {
        self.dev.interface.bus.write(&[0xbf, 0, 0]).await.unwrap();
        Ok(())
    }

    /// Case temperature in Celsius.
    pub async fn tcase(&mut self) -> Result<f32, DeviceError<BUS, CS>> {
        let adc = self.dev.adc_9_sr().read_async().await?.tcase() as f32;
        Ok(401.8 - (1.009 * adc))
    }

    /// Enter normal mode.
    pub async fn enter_normal(&mut self) -> Result<(), DeviceError<BUS, CS>> {
        self.dev.ctrl().write_async(|w| w.set_unlock(true)).await?;
        self.dev
            .ctrl()
            .write_async(|w| {
                w.set_en(true);
                w.set_gostby(false);
            })
            .await?;
        Ok(())
    }

    /// Enter standby mode.
    pub async fn enter_standby(&mut self) -> Result<(), DeviceError<BUS, CS>> {
        self.dev.ctrl().write_async(|w| w.set_unlock(true)).await?;
        self.dev
            .ctrl()
            .write_async(|w| {
                w.set_en(false);
                w.set_gostby(true);
            })
            .await?;
        Ok(())
    }

    /// Toggle watchdog bit.
    pub async fn toggle_watchdog(&mut self) -> Result<(), DeviceError<BUS, CS>> {
        self.dev
            .socr()
            .modify_async(|w| {
                w.set_wdtb(true);
            })
            .await?;
        self.dev
            .socr()
            .modify_async(|w| {
                w.set_wdtb(false);
            })
            .await
    }

    /// Control the internal pull-up current generator to detect open-load in the off state.
    pub async fn output_pull_up(
        &mut self,
        num: usize,
        on: bool,
    ) -> Result<(), DeviceError<BUS, CS>> {
        assert!(num < self.channels, "Channel number outside of bounds");

        self.dev
            .out_ctr_cr(num)
            .modify_async(|w| {
                w.set_oloffcr(on);
            })
            .await
    }

    /// Control an output.
    ///
    /// # Panics
    /// - `num` must be less than the number of channels.
    /// - `duty` must be less than 1024.
    pub async fn output(
        &mut self,
        num: usize,
        on: bool,
        duty: u16,
    ) -> Result<(), DeviceError<BUS, CS>> {
        assert!(num < self.channels, "Channel number outside of bounds");
        assert!(duty < 1024, "Duty maximum is 1023");

        self.dev
            .socr()
            .modify_async(|r| match num {
                0 => r.set_socr_0(on),
                1 => r.set_socr_1(on),
                2 => r.set_socr_2(on),
                3 => r.set_socr_3(on),
                4 => r.set_socr_4(on),
                5 => r.set_socr_5(on),
                _ => unreachable!(),
            })
            .await?;

        self.dev
            .out_ctr_cr(num)
            .write_async(|w| w.set_duty(duty))
            .await?;

        Ok(())
    }

    pub async fn output_enabled(&mut self, num: usize) -> Result<bool, DeviceError<BUS, CS>> {
        assert!(num < self.channels, "Channel number outside of bounds");

        let result = self.dev.socr().read_async().await?;

        Ok(match num {
            0 => result.socr_0(),
            1 => result.socr_1(),
            2 => result.socr_2(),
            3 => result.socr_3(),
            4 => result.socr_4(),
            5 => result.socr_5(),
            _ => unreachable!(),
        })
    }

    /// Configure the channel kind.
    pub async fn channel_kind(
        &mut self,
        num: usize,
        kind: ChannelKind,
    ) -> Result<(), DeviceError<BUS, CS>> {
        assert!(num < self.channels, "Channel number outside of bounds");

        self.dev
            .out_cfg_r(num)
            .modify_async(|w| w.set_ccr(kind))
            .await
    }

    /// Configure the frequency divisor.
    pub async fn pwm_divisor(
        &mut self,
        num: usize,
        divisor: PwmFreq,
    ) -> Result<(), DeviceError<BUS, CS>> {
        assert!(num < self.channels, "Channel number outside of bounds");

        self.dev
            .out_cfg_r(num)
            .modify_async(|w| w.set_pwmfcy(divisor))
            .await
    }

    /// Current sample point selection.
    pub async fn current_sample_point(
        &mut self,
        num: usize,
        sp: CurrentSamplePoint,
    ) -> Result<(), DeviceError<BUS, CS>> {
        assert!(num < self.channels, "Channel number outside of bounds");

        self.dev
            .out_cfg_r(num)
            .modify_async(|w| w.set_spcr(sp))
            .await
    }

    /// Channel phase.
    pub async fn channel_phase(
        &mut self,
        num: usize,
        phase: u8,
    ) -> Result<(), DeviceError<BUS, CS>> {
        assert!(num < self.channels, "Channel number outside of bounds");
        assert!(phase < 32, "Maximum phase value is 32");

        self.dev
            .out_cfg_r(num)
            .modify_async(|w| w.set_chpha(phase))
            .await
    }

    /// Slope control.
    pub async fn slope_control(
        &mut self,
        num: usize,
        slope: SlopeControl,
    ) -> Result<(), DeviceError<BUS, CS>> {
        assert!(num < self.channels, "Channel number outside of bounds");

        self.dev
            .out_cfg_r(num)
            .modify_async(|w| w.set_slopecr(slope))
            .await
    }

    /// Output power limitation behaviour.
    pub async fn off_time(
        &mut self,
        num: usize,
        latch_time: LatchOffTime,
    ) -> Result<(), DeviceError<BUS, CS>> {
        assert!(num < self.channels, "Channel number outside of bounds");

        let time = latch_time as u8;

        match num {
            0 => {
                self.dev
                    .chl_off_tcr_0()
                    .modify_async(|w| w.set_chlofftcr_0(time))
                    .await?
            }
            1 => {
                self.dev
                    .chl_off_tcr_0()
                    .modify_async(|w| w.set_chlofftcr_1(time))
                    .await?
            }
            2 => {
                self.dev
                    .chl_off_tcr_0()
                    .modify_async(|w| w.set_chlofftcr_2(time))
                    .await?
            }
            3 => {
                self.dev
                    .chl_off_tcr_1()
                    .modify_async(|w| w.set_chlofftcr_3(time))
                    .await?
            }
            4 => {
                self.dev
                    .chl_off_tcr_1()
                    .modify_async(|w| w.set_chlofftcr_4(time))
                    .await?
            }
            5 => {
                self.dev
                    .chl_off_tcr_1()
                    .modify_async(|w| w.set_chlofftcr_5(time))
                    .await?
            }
            _ => unreachable!(),
        }

        Ok(())
    }

    /// Current sense value.
    pub async fn current_sense(&mut self, num: usize) -> Result<(f32, bool), DeviceError<BUS, CS>> {
        assert!(num < self.channels, "Channel number outside of bounds");

        let read = self.dev.adc_sr(num).read_async().await?;

        // gain values as per the datasheet
        let gain = match self.channels {
            // VN9D5D20F
            4 => match num {
                0 | 1 => 34.0,
                2 | 3 => 83.0,
                _ => unreachable!(),
            },
            // VN9E30F
            6 => 107.0,
            _ => unreachable!(),
        };

        let sense = read.adcsr() as f32 / gain;

        Ok((sense, read.updtsr()))
    }

    /// PWM triggering mode.
    pub async fn pwm_trig(&mut self, trig: PwmTrigger) -> Result<(), DeviceError<BUS, CS>> {
        self.dev
            .ctrl()
            .modify_async(|r| r.set_pwm_trig(trig))
            .await?;

        Ok(())
    }

    /// Clear the internal PWM counter restarting the timing of all channels.
    pub async fn pwm_sync(&mut self) -> Result<(), DeviceError<BUS, CS>> {
        self.dev.ctrl().modify_async(|r| r.set_pwmsync(true)).await
    }
}

/// Latch-off time.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt-1", derive(defmt::Format))]
pub enum LatchOffTime {
    LatchOff = 0x0,
    Time16ms = 0x1,
    Time32ms = 0x2,
    Time48ms = 0x3,
    Time64ms = 0x4,
    Time80ms = 0x5,
    Time96ms = 0x6,
    Time112ms = 0x7,
    Time128ms = 0x8,
    Time144ms = 0x9,
    Time160ms = 0xA,
    Time176ms = 0xB,
    Time192ms = 0xC,
    Time208ms = 0xD,
    Time224ms = 0xE,
    Time240ms = 0xF,
}

device_driver::create_device!(
    device_name: Vn9e30f,
    dsl: {
        config {
            type RegisterAddressType = u8;
        }

        /// Output control register
        register OutCtrCr {
            type Access = RW;
            type ByteOrder = BE;
            const ADDRESS = 0x00;
            const SIZE_BITS = 16;
            const REPEAT = {
              count: 6,
              stride: 1,
            };

            /// WDTB: watchdog toggle bit
            wdtb: bool = 1,
            /// OLOFFCR: enables an internal pull-up current generator to
            /// distinguish between the two faults: open-load OFF-state vs the
            /// output shorted to VCC fault.
            oloffcr: bool = 2,
            /// DUTY_CR[9:0]: set the duty cycle value. Bit 9 (MSB) - Bit 0 (LSB)
            duty: uint = 4..=13,
        },

        /// Output configuration register
        register OutCfgR {
            type Access = RW;
            type ByteOrder = BE;
            const ADDRESS = 0x08;
            const SIZE_BITS = 16;
            const REPEAT = {
                count: 6,
                stride: 1,
            };

            /// VDSMASK: VDS detection at turn-off masking bit
            vdsmask: bool = 1,
            /// DIENCR: Direct input signal enable in normal mode (according to OTP allocation)
            diencr: bool = 2,
            /// CCR: Set the channel configuration (BULB-LED)
            ccr: uint as enum ChannelKind {
                Bulb = 0,
                Led = 1,
            } = 3..=3,
            /// PWMFCY[1:0]: PWM frequency selection
            pwmfcy: uint as enum PwmFreq {
                Ratio1024 = 0b00,
                Ratio2048 = 0b01,
                Ratio4096 = 0b10,
                Ratio512 = 0b11,
            } = 4..=5,
            /// SPCR[1:0]: Current sampling point
            spcr: uint as enum CurrentSamplePoint {
                Stop = 0b00,
                Start = 0b01,
                Continuous = 0b10,
                Filtered = 0b11,
            } = 6..=7,
            /// CHPHA[4:0]: Set the Channel phase value
            chpha: uint = 8..=12,
            /// SLOPECR[1:0]: Switching slope control bit 1 (MSB) and 0 (LSB)
            slopecr: uint as enum SlopeControl {
                Standard = 0b00,
                Fast = 0b01,
                Faster = 0b10,
                Fastest = 0b11,
            } = 14..=15,
        },

        /// Channel latch OFF timer control register
        register ChlOffTcr0 {
            type Access = RW;
            type ByteOrder = BE;
            const ADDRESS = 0x10;
            const SIZE_BITS = 16;

            /// CHLOFFTCR: Configure the output behavior in case of power
            /// limitation for the corresponding channel 0.
            chlofftcr0: uint = 4..=7,
            /// CHLOFFTCR: Configure the output behavior in case of power
            /// limitation for the corresponding channel 1.
            chlofftcr1: uint = 8..=11,
            /// CHLOFFTCR: Configure the output behavior in case of power
            /// limitation for the corresponding channel 2.
            chlofftcr2: uint = 12..=15,
        },

        /// Channel latch OFF timer control register
        register ChlOffTcr1 {
            type Access = RW;
            type ByteOrder = BE;
            const ADDRESS = 0x11;
            const SIZE_BITS = 16;

            /// CHLOFFTCR: Configure the output behavior in case of power
            /// limitation for the corresponding channel 3.
            chlofftcr3: uint = 4..=7,
            /// CHLOFFTCR: Configure the output behavior in case of power
            /// limitation for the corresponding channel 4.
            chlofftcr4: uint = 8..=11,
            /// CHLOFFTCR: Configure the output behavior in case of power
            /// limitation for the corresponding channel 4.
            chlofftcr5: uint = 12..=15,
        },

        /// Channel control register
        register Socr {
            type Access = RW;
            type ByteOrder = BE;
            const ADDRESS = 0x13;
            const SIZE_BITS = 16;

            /// Watchdog toggle bit
            wdtb: bool = 1,
            /// SOCR0: bit controls output state of channel 0
            socr0: bool = 8,
            /// SOCR1: bit controls output state of channel 1
            socr1: bool = 9,
            /// SOCR2: bit controls output state of channel 2
            socr2: bool = 10,
            /// SOCR3: bit controls output state of channel 3
            socr3: bool = 11,
            /// SOCR4: bit controls output state of channel 4
            socr4: bool = 12,
            /// SOCR5: bit controls output state of channel 5
            socr5: bool = 13,
        },

        /// Control register
        register Ctrl {
            type Access = RW;
            type ByteOrder = BE;
            const ADDRESS = 0x14;
            const SIZE_BITS = 16;

            /// PWMSYNC: PWM clock synchronisation
            pwmsync: bool = 1,
            /// LOCKEN: Protected transaction mode
            locken: uint = 2..=6,
            /// PWM_TRIG: PWM triggering mode
            pwm_trig: uint as enum PwmTrigger {
                RisingEdge = 0,
                FallingEdge = 1,
            } = 10..=10,
            /// EN: Enter normal mode
            en: bool = 11,
            /// CTDTH: Case thermal detection threshold
            ctdth: uint as enum ThermalThreshold {
                Temp120C = 0b00,
                Temp130C = 0b01,
                Temp140C = 0b10,
                Temp140C2 = 0b11,
            } = 12..=13,
            /// UNLOCK: Allows protected SPI transactions
            unlock: bool = 14,
            /// GOSTBY: Go to standby
            gostby: bool = 15,
        },

        /// Output status register
        register OutSr {
            type Access = RW;
            type ByteOrder = BE;
            const ADDRESS = 0x20;
            const SIZE_BITS = 16;
            const REPEAT = {
                count: 6,
                stride: 1,
            };

            /// VCCUV: Vcc undervoltage
            vccuv: bool = 4,
            /// PWMCLOCKLOW: PWM clock frequency too low
            pwmclocklow: bool = 5,
            /// SPIE: SPI error
            spie: bool = 6,
            /// RST: Chip reset
            rst: bool = 7,
            /// CHLOFFSRx: Channel latch-off status
            chloffsr: bool = 8,
            /// OLPUSRx: Output pull-up generator status
            olpusr: bool = 9,
            /// STKFLTRx: Output stuck to Vcc/open-load off state status
            stkfltr: bool = 10,
            /// VDSFSRx: VDS feedback status
            vdsfsr: bool = 11,
            /// CHFBSTx: Channnel feedback status
            chfbst: bool = 12,
            /// DIOTP0: Associated DIx input description bit 0
            diotp0: bool = 13,
            /// DIOTP1: Associated DIx input description bit 1
            diotp1: bool = 14,
            /// DIENSR: Direct input status, image of associated DI logic level according to OTP allocation
            diensr: bool = 15,
        },

        /// Digital current sense register
        register AdcSr {
            type Access = RW;
            type ByteOrder = BE;
            const ADDRESS = 0x28;
            const SIZE_BITS = 16;
            const REPEAT = {
                count: 6,
                stride: 1,
            };

            /// UPDTSR: updated status bit
            updtsr: bool = 1,
            /// SOCRx: output state of channel
            socr: bool = 2,
            /// ADCxSR: the 10-bit digital value of output current
            adcsr: uint = 4..=13,
        },

        /// Digital case thermal sensor voltage register
        register Adc9Sr {
            type Access = RW;
            type ByteOrder = BE;
            const ADDRESS = 0x31;
            const SIZE_BITS = 16;

            /// UPDTSR: updated status bit
            updtsr: bool = 1,
            /// ADC9SR: the 10-bit digital value of the temperature sensor voltage
            tcase: int = 4..=13,
        },
    }
);
