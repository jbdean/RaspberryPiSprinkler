#!/usr/bin/perl
#
# grab valve states
# 0 2 3 4 5 6 25 27

$valve = `./gpio read 0`;
$valve .= `./gpio read 2`;
$valve .= `./gpio read 3`;
$valve .= `./gpio read 4`;
$valve .= `./gpio read 5`;
$valve .= `./gpio read 6`;
$valve .= `./gpio read 25`;
$valve .= `./gpio read 27`;

print "$valve";

