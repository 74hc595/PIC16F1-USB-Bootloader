#!/bin/bash
OUTFILE="import_list.inc"

echo $OUTFILE
echo '; Exported USB Descriptor Addresses' >  $OUTFILE
echo '; ' >>$OUTFILE
echo '; See export_list for the list of addresses to export' >>$OUTFILE
echo '; Creates file import_list.inc which should be used by ' >>$OUTFILE
echo '; the application code.' >>$OUTFILE
echo '' >>$OUTFILE
echo 'if USE_BOOTLOADER_CDC' >>$OUTFILE

while read srchstr; do 

   grep "$srchstr" *.lst | awk '{if (NF==3) print $3 "\tequ\t0x" $1}'

done < export_list >> $OUTFILE
echo 'endif' >> $OUTFILE
echo '' >> $OUTFILE
grep "BOOTLOADER_SIZE" *.lst | awk '{if ((NF==5) && ($4=="equ")) print $3 "\tequ\t" $5}' >> $OUTFILE

