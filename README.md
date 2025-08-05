# Driver for ST SPI High Side Switches

This is a driver crate for the [VN9E30F](https://www.st.com/en/automotive-analog-and-power/vn9e30f.html). All SPI transactions are implemented atop of `embedded-hal-async` traits, so this should work with any HAL that implements those traits.

Although the VN9E30F is the only supported driver right now, it should work with any other 6-channel driver in the same family. Likewise for the 4-channel drivers so long as channels 5 and 6 aren't used.
