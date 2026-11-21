/* nRF52833: 512 KiB flash, 128 KiB RAM. Included by cortex-m-rt's link.x. */
MEMORY
{
  FLASH : ORIGIN = 0x00000000, LENGTH = 512K
  RAM   : ORIGIN = 0x20000000, LENGTH = 128K
}
