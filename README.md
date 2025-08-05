# Driver for ST SPI High Side Switches

This is a driver crate for the [VN9E30F](https://www.st.com/en/automotive-analog-and-power/vn9e30f.html). All SPI transactions are implemented atop of `embedded-hal-async` traits, so this should work with any HAL that implements those traits.
