## Undergraduate Teaching Platform: for the [TI TM4C Launchpad](http://www.ti.com/tool/EK-TM4C123GXL)! 👷

[![Build Status](https://img.shields.io/endpoint.svg?url=https%3A%2F%2Factions-badge.atrox.dev%2Fut-utp%2Ftm4c%2Fbadge&style=for-the-badge)](https://github.com/ut-utp/tm4c/actions) [![License: MPL-2.0](https://img.shields.io/github/license/ut-utp/tm4c?color=orange&style=for-the-badge)](https://opensource.org/licenses/MPL-2.0)
--
[![](https://tokei.rs/b1/github/ut-utp/m4c)](https://github.com/ut-utp/tm4c) [![codecov](https://codecov.io/gh/ut-utp/tm4c/branch/master/graph/badge.svg)](https://codecov.io/gh/ut-utp/tm4c)

Uses [thejpster](https://github.com/thejpster/)'s [tm4c-hal crates](https://github.com/thejpster/tm4c-hal) heavily.

🐝 🚧 This is very much not stable yet! 🚧 🐝

## Usage

To be used in conjuction with the [UTP TUI](//github.com/ut-utp/tui.git).

#### First: Flash your TM4C

You can grab a TM4C image (a `.bin` file; TODO: issue #5) from the [releases page](https://github.com/ut-utp/tm4c/releases).

You'll need to grab `lm4flash` and potentially install a driver in order to flash your TM4C. [This page](https://github.com/ut-utp/.github/wiki/Dev-Environment-Setup#for-the-tm4c) has instructions on how to do so.

(TODO: replace with `utp-tui --flash tm4c`; flip the sections around so that installing the TUI comes first!)
Once you've done this, to flash your board run `lm4flash -v <path to the .bin file>`.

On macOS and Linux:
  - `lm4flash -v utp-tm4c.bin`
On Windows:
  - `lm4flash.exe -v utp-tm4c.bin`

At this point, if flashing the board was successful, your on-board LED should be blinking (TODO: issue #4).

(TODO: ulimately we want to switch to probe-rs and have the TUI handle this, actually...)
(See: ut-utp/tui#11)

#### Next: Launch the TUI

First [install](https://github.com/ut-utp/tui#usage) the UTP TUI if you haven't already.

Next, find your device's serial port:
  - Windows: open device manager, look for COM ports, find the one that says stellaris
  - macOS: look in `/dev/` for something that starts with `/dev/cu.usbmodem`
  - Linux: `dmesg | tail` after you plug in or look in `/dev/` (probably something like `/dev/ttyACM0` if you don't have the [udev rule](https://github.com/ut-utp/.github/wiki/Dev-Environment-Setup#for-the-tm4c); otherwise `/dev/tm4c`)

And finally, run the TUI with the `--device board=<serial port path>:1500000` flag.

For example:
  - Windows: `utp-tui.exe --device board=COM11:1500000`
  - macOS: `./utp-tui --device board=/dev/cu.usbmodemABCD1234:1500000`
  - Linux: `./utp-tui --device board=/dev/tm4c:1500000`

(TODO: ultimately we want to streamline this to just `utp-tui --device tm4c`, tui#6)

## Development

### Setup

If you're looking to make changes to or hack on `utp-tm4c` there are a few more things you'll need to set up!

#### Using Nix

> Note: If you're using macOS or Linux we _strongly_ recommend [using `nix` and VSCode](https://github.com/ut-utp/.github/wiki/Dev-Environment-Setup#using-nix). This is the best supported workflow.
#### Otherwise

Follow [these instructions](https://github.com/ut-utp/.github/wiki/Dev-Environment-Setup) to set up your dev environment. Be sure to also follow the ["Embedded Development Setup" instructions](https://github.com/ut-utp/.github/wiki/Dev-Environment-Setup#embedded-development-setup).

### Doing Development

TODO: VSCode aliases
TODO: targets for all of these..

To build the project:
  - `cargo b` to build just the main program
  - `cargo ba` to build everything

`rustup` will automatically fetch the [appropriate toolchain](rust-toolchain.toml) and the tools needed to build for ARM.

To flash your board (without attaching a debugger):
  - `cargo f`
    + This uses `lm4flash` and attempts to grab `lm4flash` binaries from [here](https://github.com/ut-utp/.github/wiki/lm4flash-Binaries) if you do not already have it installed locally.

To run a program that has console output:
  - `cargo r --bin <bin name>` for binaries
  - `cargo r --example <example name>` for examples
  - this will drop you into `picocom` if you have it installed (TODO)


To debug the project (launches a debugger):
  - `cargo d`
    + This uses `gdb` as a "runner" as specified in [`.cargo/config`](.cargo/config).
    + If you're not using `nix`, make sure you have installed GDB, OpenOCD, and the LLVM binutils as [described in the setup guide](https://github.com/ut-utp/.github/wiki/Dev-Environment-Setup#embedded-development-setup).
    + You may need to [configure GDB](https://sourceware.org/gdb/onlinedocs/gdb/Auto_002dloading-safe-path.html) to allow autoloading [`.gdbinit`](.gdbinit) (optional).

> 🚨 NOTE 🚨: Be sure to run `continue` before exiting GDB! If you do not do this your board will enter a bad state and will need to be power cycled before you can attach a debugger to it again.

To run the tests:
  - `cargo t` to run all tests (with assertions)
  - `cargo tr` to run all tests in release mode
  - `cargo test --test <test name> --target thumbv7em-none-eabihf -- --test` to run a specific test (add `--release` before the `--` for release mode)

To run the benchmarks:
  - `cargo ben` to run all the benchmarks
  - `cargo ben --bench <benchmark name>` to run a specific benchmark

#### VSCode Support

This project also has a [VSCode workspace](.vscode/tm4c.code-workspace) that is configured with build and debug support for this project.

To build:
  - the [default build task](.vscode/tasks.json) builds the project
  - trigger this with:
    * <kbd>ctrl</kbd> + <kbd>shift</kbd> + <kbd>b</kbd> on Windows/Linux
    * <kbd>cmd</kbd> + <kbd>shift</kbd> + <kbd>b</kbd> on macOS

To run/debug the project (launches VSCode's integrated debugger):
  - the [default launch configuration](.vscode/launch.json) builds the project and starts a debugging session
  - trigger this by pressing <kbd>f5</kbd>
    * or focus on the Debug tab in the sidebar and press the green play button
  - **Note**: Windows users will need to modify [launch.json](.vscode/launch.json) to make reference to Windows executables for `gdb`, `openocd`, and `llvm-objdump`
    * i.e. replace `gdb` with `gdb.exe`, etc.
  - **Note**: if you are **not** using `nix` you will need to manually [fetch `TM4C123GH6PM.svd`](https://github.com/ut-utp/tm4c/blob/55d67a9ed04ea08bdffff7abedfe8f70a349df23/flake.nix#L76-L79) and place it at `.vscode/TM4C123GH6PM.svd` if you want to use the device register views that the [Cortex-Debug extension](https://github.com/Marus/cortex-debug/wiki/Overview) offers
    * i.e. `curl -L https://raw.githubusercontent.com/posborne/cmsis-svd/master/data/TexasInstruments/TM4C123GH6PM.svd > .vscode/TM4C123GH6PM.svd`

> 🚨 NOTE 🚨: Be sure to `continue` (<kbd>F5</kbd>) before exiting your debug session! If you do not do this your board will enter a bad state and will need to be power cycled before you can attach a debugger to it again.

(TODO: tests, test task, etc.)
