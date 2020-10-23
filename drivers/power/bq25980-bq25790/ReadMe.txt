1, Use "git am" or "git apply" to merge "0001-PD-charge-for-bq25980-and-bq25790.patch" to your kernel. If you got git errors, you can merge it manually.
2, Copy "ti" folder to "drivers/power/supply/".
3, Add "bq25790@6b" and "bq25980@65" to your dts file as "kona-hdk-overlay.dts".