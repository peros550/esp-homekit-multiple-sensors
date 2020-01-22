
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
//#include <task.h>
#include <ir/ir.h>
#include <ir/raw.h>
#include "ac_commands.h"

//Below codes are for TOYOTOMI AC
 int16_t Fan[] = {9016 ,-4462 ,  680 ,-1628 ,  681 ,-1627 ,  681 , -551 ,  656 ,-1626 ,  682 , -551 ,  656 , -550 ,  657 , -550 ,
646 , -559 ,  648 ,-1634 ,  675 , -557 ,  650 , -556 ,  650 ,-1631 ,  678 , -555 ,  652 , -555 ,  652 , -554 ,
653 , -553 ,  653 , -553 ,  654 , -552 ,  655 , -551 ,  656 , -551 ,  656 , -550 ,  656 ,-1625 ,  673 ,-1633 ,
676 , -558 ,  649 , -556 ,  651 , -556 ,  650 , -557 ,  650 , -556 ,  651 ,-1630 ,  678 , -554 ,  653 ,-1628 ,
681 , -551 ,  656 , -551 ,  656 ,-1625 ,  683 , -550 ,  657 ,-9999,-9955 ,  675 , -558 ,  649 , -557 ,  649 , -557 ,
650 , -557 ,  650 , -555 ,  652 , -555 ,  652 , -554 ,  652 , -554 ,  653 , -553 ,  654 , -552 ,  655 , -551 ,
656 , -550 ,  657 , -550 ,  656 ,-1625 ,  684 , -549 ,  647 , -559 ,  648 , -558 ,  649 , -558 ,  649 , -556 ,
650 , -557 ,  650 , -556 ,  651 , -556 ,  650 , -556 ,  651 , -556 ,  651 , -581 ,  657 , -549 ,  658 , -549 ,
647 , -560 ,  647 , -559 ,  648 , -558 ,  648 , -558 ,  649 , -558 ,  649}; //AnalysIR Batch Export (IRremote) - RAW
 int16_t Cool_16[] = {9013 ,-4467 ,  680 ,-1629 ,  682 , -525 ,  683 , -523 ,  674 ,-1633 ,  678 ,-1630 ,  681 ,-1627 ,  684 , -522 ,
675 , -532 ,  676 , -531 ,  677 , -528 ,  680 , -527 ,  681 , -525 ,  673 , -532 ,  676 , -531 ,  677 , -529 ,
679 , -527 ,  681 , -525 ,  683 , -524 ,  684 , -523 ,  674 , -532 ,  676 , -531 ,  677 ,-1630 ,  681 ,-1628 ,
683 , -524 ,  674 , -532 ,  676 , -530 ,  678 , -528 ,  680 , -527 ,  681 ,-1626 ,  684 , -523 ,  675 ,-1632 ,
679 , -528 ,  680 , -527 ,  681 ,-1626 ,  685 , -522 ,  675 ,-9999,-9965 ,  674 , -533 ,  675 ,-1632 ,  679 , -528 ,
680 , -527 ,  681 , -525 ,  683 , -523 ,  675 , -532 ,  676 , -530 ,  678 , -528 ,  680 , -527 ,  681 , -525 ,
683 , -524 ,  684 , -523 ,  674 ,-1633 ,  678 , -555 ,  654 , -526 ,  682 , -525 ,  682 , -525 ,  683 , -523 ,
675 , -532 ,  676 , -531 ,  677 , -530 ,  678 , -554 ,  654 , -526 ,  682 , -524 ,  674 , -532 ,  676 , -530 ,
678 , -529 ,  679 ,-1629 ,  681 , -525 ,  684 ,-1624 ,  676 , -530 ,  678 };

 int16_t Cool_17[] = {9050 ,-4429 ,  706 ,-1602 ,  708 , -499 ,  709 , -497 ,  711 ,-15 ,  704 ,-1604 ,  706 , -500 ,
-496 ,  712 , -493 ,  704 , -503 ,  704 , -502 ,  706 , -500 ,  707 ,-1601 ,  709 ,-1599 ,
712 , -493 ,  704 , -503 ,  704 , -502 ,  706 , -500 ,  708 , -498 ,  709 ,-1600 ,  710 , -496 ,  712 ,-1596 ,
703 , -503 ,  705 , -502 ,  706 ,-1602 ,  708 , -497 ,  710 ,-9999,-9931 ,  703 , -503 ,  705 ,-1602 ,  708 , -499 ,
708 , -498 ,  710 , -496 ,  711 , -495 ,  713 , -494 ,  703 , -503 ,  705 , -501 ,  706 , -500 ,  708 , -498 ,
710 , -496 ,  711 , -496 ,  712 ,-1595 ,  705 , -502 ,  705 , -500 ,  708 , -498 ,  709 , -498 ,  710 , -495 ,
713 , -494 ,  713 , -493 ,  704 , -502 ,  706 , -501 ,  706 , -500 ,  708 , -498 ,  710 , -496 ,  711 , -495 ,
713 , -493 ,  704 , -503 ,  705 ,-1603 ,  707 ,-1601 ,  709 , -497 ,  711 
}; //AnalysIR Batch Export (IRremote) - RAW
 int16_t Cool_18[] = {
9043 ,-4463 ,  681 ,-1626 ,  684 , -524 ,  683 , -524 ,  683 ,-1624 ,  675 ,-1633 ,  677 , -530 ,  677 , -529 ,
678 , -528 ,  679 , -527 ,  681 ,-1626 ,  683 , -524 ,  683 , -523 ,  674 , -532 ,  676 , -532 ,  675 , -530 ,
677 , -529 ,  678 , -555 ,  653 , -527 ,  680 , -526 ,  681 , -525 ,  682 , -525 ,  672 ,-1634 ,  676 ,-1631 ,
678 , -530 ,  678 , -528 ,  679 , -527 ,  680 , -526 ,  681 , -525 ,  682 ,-1625 ,  674 , -532 ,  676 ,-1632 ,
677 , -529 ,  678 , -528 ,  680 ,-1628 ,  681 , -526 ,  681 ,-9999,-9960 ,  678 , -528 ,  679 ,-1628 ,  682 , -524 ,
683 , -524 ,  683 , -524 ,  683 , -523 ,  674 , -558 ,  649 , -532 ,  675 , -531 ,  677 , -528 ,  679 , -528 ,
679 , -553 ,  654 , -526 ,  682 ,-1625 ,  684 , -523 ,  674 , -532 ,  675 , -532 ,  675 , -530 ,  678 , -529 ,
678 , -528 ,  679 , -528 ,  679 , -528 ,  680 , -526 ,  681 , -525 ,  682 , -524 ,  683 , -550 ,  647 , -533 ,
675 , -531 ,  676 ,-1631 ,  679 ,-1629 ,  680 ,-1629 ,  680 , -527 ,  681 
}; //AnalysIR Batch Export (IRremote) - RAW
 int16_t Cool_19[] = {9046 ,-4459 ,  675 ,-1633 ,  676 , -557 ,  650 , -556 ,  652 ,-1629 ,  680 ,-1627 ,  683 , -550 ,  657 , -523 ,
674 , -559 ,  648 ,-1633 ,  677 ,-1630 ,  679 , -554 ,  654 , -526 ,  681 , -552 ,  655 , -551 ,  657 , -549 ,
648 , -559 ,  648 , -558 ,  649 , -557 ,  650 , -556 ,  651 , -555 ,  653 , -553 ,  654 ,-1627 ,  682 ,-1626 ,
673 , -560 ,  648 , -558 ,  649 , -531 ,  676 , -556 ,  652 , -554 ,  653 ,-1627 ,  683 , -550 ,  657 ,-1625 ,
674 , -559 ,  648 , -532 ,  675 ,-1632 ,  678 , -555 ,  652 ,-9999,-9962 ,  676 , -556 ,  651 ,-1631 ,  679 , -554 ,
653 , -553 ,  654 , -552 ,  655 , -552 ,  656 , -523 ,  673 , -559 ,  649 , -558 ,  649 , -557 ,  651 , -556 ,
651 , -529 ,  678 , -554 ,  653 ,-1627 ,  683 , -551 ,  656 , -550 ,  657 , -550 ,  647 , -559 ,  648 , -558 ,
649 , -558 ,  650 , -556 ,  651 , -556 ,  651 , -555 ,  653 , -527 ,  680 , -553 ,  654 , -553 ,  654 , -553 ,
655 , -551 ,  656 , -551 ,  656 , -550 ,  647 , -559 ,  648 ,-1633 ,  677}; //AnalysIR Batch Export (IRremote) - RAW
 int16_t Cool_20[] = {9018 ,-4461 ,  682 ,-1626 ,  684 , -549 ,  647 , -559 ,  649 ,-1632 ,  677 ,-1630 ,  680 , -553 ,  654 , -553 ,
654 , -552 ,  656 , -550 ,  657 , -550 ,  657 ,-1624 ,  675 , -557 ,  651 , -556 ,  651 , -556 ,  651 , -555 ,
652 , -555 ,  653 , -553 ,  654 , -553 ,  655 , -551 ,  656 , -551 ,  656 , -550 ,  657 ,-1624 ,  675 ,-1632 ,
678 , -556 ,  651 , -555 ,  652 , -554 ,  653 , -553 ,  655 , -551 ,  656 ,-1625 ,  674 , -559 ,  648 ,-1632 ,
678 , -556 ,  651 , -555 ,  652 ,-1629 ,  681 , -551 ,  656 ,-9999,-9958 ,  680 , -553 ,  654 ,-1627 ,  682 , -552 ,
656 , -550 ,  647 , -533 ,  674 , -559 ,  648 , -558 ,  649 , -557 ,  651 , -555 ,  652 , -554 ,  653 , -527 ,
680 , -553 ,  655 , -551 ,  656 ,-1625 ,  674 , -559 ,  648 , -559 ,  648 , -558 ,  650 , -556 ,  651 , -556 ,
651 , -556 ,  651 , -555 ,  652 , -555 ,  653 , -553 ,  654 , -552 ,  655 , -552 ,  656 , -549 ,  647 , -560 ,
648 , -558 ,  649 ,-1632 ,  678 , -555 ,  652 , -554 ,  653 ,-1628 ,  681}; //AnalysIR Batch Export (IRremote) - RAW
 int16_t Cool_21[] = {9042 ,-4464 ,  680 ,-1627 ,  683 , -551 ,  656 , -523 ,  674 ,-1633 ,  676 ,-1631 ,  679 , -554 ,  653 , -554 ,
653 , -554 ,  654 ,-1627 ,  682 , -550 ,  657 ,-1624 ,  675 , -558 ,  650 , -531 ,  676 , -556 ,  651 , -556 ,
652 , -554 ,  653 , -554 ,  653 , -554 ,  653 , -553 ,  655 , -551 ,  656 , -551 ,  656 ,-1624 ,  675 ,-1633 ,
677 , -556 ,  651 , -556 ,  652 , -554 ,  653 , -553 ,  654 , -551 ,  656 ,-1625 ,  674 , -559 ,  649 ,-1632 ,
677 , -556 ,  651 , -555 ,  652 ,-1629 ,  681 , -552 ,  655 ,-9999,-9959 ,  678 , -555 ,  653 ,-1628 ,  681 , -553 ,
654 , -553 ,  655 , -551 ,  656 , -551 ,  656 , -550 ,  657 , -550 ,  647 , -560 ,  647 , -559 ,  649 , -557 ,
650 , -530 ,  677 , -555 ,  652 ,-1630 ,  680 , -552 ,  655 , -552 ,  655 , -551 ,  656 , -550 ,  647 , -559 ,
649 , -557 ,  650 , -556 ,  651 , -556 ,  651 , -554 ,  654 , -552 ,  655 , -552 ,  655 , -551 ,  656 , -550 ,
647 , -560 ,  647 , -558 ,  649 ,-1632 ,  678 , -555 ,  652 ,-1629 ,  681 }; //AnalysIR Batch Export (IRremote) - RAW
 int16_t Cool_22[] = {9024 ,-4456 ,  678 ,-1629 ,  680 , -553 ,  654 , -552 ,  656 ,-1625 ,  684 ,-1625 ,  674 , -558 ,  649 , -557 ,
651 , -556 ,  651 , -556 ,  651 ,-1629 ,  680 ,-1629 ,  681 , -552 ,  655 , -552 ,  655 , -552 ,  656 , -550 ,
657 , -549 ,  647 , -559 ,  649 , -557 ,  650 , -556 ,  651 , -555 ,  652 , -554 ,  654 ,-1627 ,  682 ,-1626 ,
683 , -550 ,  647 , -559 ,  648 , -558 ,  650 , -557 ,  650 , -555 ,  652 ,-1630 ,  680 , -553 ,  654 ,-1627 ,
682 , -551 ,  657 , -549 ,  647 ,-1634 ,  676 , -557 ,  650 ,-9999,-9964 ,  673 , -560 ,  648 ,-1633 ,  676 , -557 ,
650 , -530 ,  678 , -555 ,  652 , -553 ,  654 , -552 ,  655 , -551 ,  657 , -550 ,  657 , -549 ,  648 , -559 ,
648 , -558 ,  649 , -557 ,  650 ,-1630 ,  680 , -554 ,  653 , -554 ,  654 , -552 ,  655 , -552 ,  655 , -552 ,
655 , -550 ,  658 , -549 ,  647 , -559 ,  649 , -558 ,  649 , -557 ,  650 , -556 ,  651 , -556 ,  652 , -554 ,
653 , -553 ,  654 ,-1627 ,  683 ,-1626 ,  683 , -550 ,  647 ,-1634 ,  676 }; //AnalysIR Batch Export (IRremote) - RAW
 int16_t Cool_23[] = {9020 ,-4459 ,  674 ,-1633 ,  677 , -557 ,  650 , -556 ,  651 ,-1630 ,  679 ,-1629 ,  681 , -552 ,  655 , -551 ,
656 , -551 ,  657 ,-1623 ,  676 ,-1632 ,  677 ,-1631 ,  679 , -553 ,  654 , -552 ,  655 , -552 ,  655 , -551 ,
657 , -549 ,  648 , -559 ,  648 , -558 ,  649 , -558 ,  649 , -557 ,  650 , -557 ,  651 ,-1629 ,  680 ,-1628 ,
682 , -552 ,  655 , -551 ,  656 , -551 ,  656 , -550 ,  647 , -559 ,  648 ,-1633 ,  677 , -556 ,  651 ,-1630 ,
679 , -554 ,  653 , -553 ,  655 ,-1626 ,  683 , -550 ,  647 ,-9999,-9965 ,  682 , -551 ,  656 ,-1624 ,  675 , -558 ,
650 , -556 ,  651 , -556 ,  651 , -555 ,  652 , -554 ,  653 , -553 ,  655 , -550 ,  657 , -549 ,  648 , -559 ,
648 , -559 ,  648 , -558 ,  649 ,-1632 ,  678 , -554 ,  653 , -554 ,  653 , -553 ,  654 , -552 ,  656 , -551 ,
656 , -550 ,  657 , -549 ,  648 , -559 ,  648 , -557 ,  650 , -556 ,  651 , -555 ,  653 , -552 ,  655 , -552 ,
655 , -552 ,  655 , -551 ,  657 , -550 ,  657 ,-1624 ,  675 ,-1632 ,  678}; //AnalysIR Batch Export (IRremote) - RAW
 int16_t Cool_24[] = {9020 ,-4458 ,  675 ,-1633 ,  677 , -556 ,  651 , -556 ,  651 ,-1629 ,  681 ,-1627 ,  682 , -551 ,  656 , -550 ,
658 , -549 ,  647 , -559 ,  649 , -557 ,  650 , -557 ,  650 ,-1630 ,  679 , -554 ,  654 , -553 ,  654 , -552 ,
655 , -551 ,  656 , -550 ,  658 , -548 ,  648 , -558 ,  649 , -557 ,  651 , -556 ,  651 ,-1630 ,  679 ,-1628 ,
682 , -551 ,  656 , -550 ,  657 , -550 ,  647 , -558 ,  649 , -558 ,  649 ,-1632 ,  678 , -555 ,  652 ,-1629 ,
680 , -553 ,  655 , -552 ,  655 ,-1625 ,  684 , -550 ,  657 ,-9999,-9954 ,  673 , -559 ,  648 ,-1633 ,  676 , -556 ,
652 , -554 ,  653 , -554 ,  653 , -553 ,  654 , -552 ,  655 , -552 ,  655 , -550 ,  658 , -549 ,  647 , -559 ,
649 , -557 ,  650 , -556 ,  651 ,-1630 ,  679 , -554 ,  654 , -553 ,  654 , -551 ,  656 , -550 ,  647 , -560 ,
647 , -558 ,  650 , -556 ,  651 , -556 ,  651 , -555 ,  652 , -555 ,  652 , -554 ,  654 , -552 ,  655 , -551 ,
656 , -551 ,  657 ,-1623 ,  675 , -559 ,  649 ,-1632 ,  677 ,-1631 ,  678}; //AnalysIR Batch Export (IRremote) - RAW
 int16_t Cool_25[] = {9020 ,-4461 ,  683 ,-1625 ,  674 , -559 ,  648 , -558 ,  649 ,-1633 ,  677 ,-1630 ,  679 , -553 ,  654 , -553 ,
654 , -553 ,  655 ,-1626 ,  683 , -550 ,  657 , -549 ,  648 ,-1633 ,  677 , -555 ,  652 , -555 ,  652 , -554 ,
653 , -554 ,  654 , -553 ,  654 , -553 ,  654 , -552 ,  655 , -551 ,  657 , -549 ,  647 ,-1634 ,  676 ,-1631 ,
678 , -556 ,  651 , -555 ,  653 , -553 ,  654 , -552 ,  655 , -551 ,  656 ,-1624 ,  675 , -559 ,  648 ,-1633 ,
677 , -556 ,  651 , -555 ,  652 ,-1629 ,  681 , -552 ,  655 ,-9999,-9959 ,  677 , -556 ,  651 ,-1630 ,  680 , -553 ,
654 , -552 ,  655 , -552 ,  656 , -550 ,  657 , -549 ,  648 , -565 ,  642 , -557 ,  650 , -557 ,  650 , -556 ,
652 , -554 ,  653 , -553 ,  654 ,-1627 ,  683 , -550 ,  657 , -549 ,  648 , -558 ,  649 , -558 ,  649 , -557 ,
650 , -555 ,  652 , -555 ,  653 , -553 ,  654 , -553 ,  654 , -552 ,  655 , -551 ,  646 , -559 ,  648 , -558 ,
650 , -557 ,  650 , -556 ,  651 ,-1630 ,  679 ,-1628 ,  681 ,-1627 ,  683}; //AnalysIR Batch Export (IRremote) - RAW
 int16_t Cool_26[] = {9017 ,-4462 ,  681 ,-1627 ,  682 , -551 ,  657 , -549 ,  647 ,-1634 ,  676 ,-1632 ,  677 , -555 ,  652 , -555 ,
652 , -555 ,  653 , -553 ,  654 ,-1627 ,  683 , -551 ,  656 ,-1625 ,  684 , -549 ,  648 , -558 ,  649 , -558 ,
649 , -558 ,  649 , -557 ,  651 , -554 ,  653 , -554 ,  653 , -552 ,  655 , -552 ,  656 ,-1626 ,  683 ,-1625 ,
684 , -550 ,  647 , -560 ,  647 , -559 ,  649 , -558 ,  649 , -557 ,  650 ,-1631 ,  678 , -557 ,  651 ,-1630 ,
679 , -554 ,  653 , -553 ,  655 ,-1626 ,  683 , -550 ,  657 ,-9999,-9957 ,  680 , -552 ,  655 ,-1627 ,  682 , -550 ,
647 , -559 ,  648 , -559 ,  648 , -559 ,  649 , -557 ,  650 , -556 ,  651 , -554 ,  653 , -554 ,  653 , -553 ,
655 , -551 ,  656 , -551 ,  656 ,-1623 ,  676 , -558 ,  649 , -558 ,  649 , -556 ,  652 , -554 ,  653 , -554 ,
653 , -554 ,  654 , -552 ,  655 , -551 ,  656 , -550 ,  647 , -559 ,  648 , -558 ,  649 , -558 ,  649 , -557 ,
651 , -555 ,  652 ,-1629 ,  680 ,-1628 ,  681 ,-1627 ,  683 ,-1625 ,  684}; //AnalysIR Batch Export (IRremote) - RAW
 int16_t Cool_27[] = {9021 ,-4457 ,  677 ,-1629 ,  680 , -554 ,  653 , -553 ,  654 ,-1627 ,  682 ,-1625 ,  674 , -561 ,  647 , -557 ,
650 , -557 ,  650 ,-1631 ,  678 ,-1629 ,  681 , -552 ,  655 ,-1626 ,  683 , -550 ,  647 , -559 ,  648 , -558 ,
650 , -556 ,  651 , -555 ,  652 , -555 ,  652 , -554 ,  653 , -553 ,  655 , -552 ,  655 ,-1625 ,  674 ,-1634 ,
675 , -558 ,  650 , -556 ,  651 , -556 ,  651 , -555 ,  652 , -554 ,  653 ,-1628 ,  682 , -551 ,  656 ,-1624 ,
675 , -558 ,  649 , -557 ,  650 ,-1631 ,  678 , -555 ,  653 ,-9999,-9959 ,  677 , -556 ,  651 ,-1630 ,  679 , -554 ,
653 , -553 ,  655 , -551 ,  656 , -550 ,  657 , -549 ,  648 , -558 ,  649 , -557 ,  650 , -556 ,  651 , -556 ,
651 , -555 ,  653 , -554 ,  653 ,-1628 ,  681 , -552 ,  655 , -551 ,  657 , -549 ,  647 , -560 ,  648 , -559 ,
648 , -558 ,  649 , -557 ,  650 , -557 ,  650 , -556 ,  652 , -554 ,  653 , -553 ,  654 , -553 ,  654 , -552 ,
656 , -550 ,  657 , -549 ,  648 , -559 ,  648 , -558 ,  649 , -557 ,  650 }; //AnalysIR Batch Export (IRremote) - RAW
 int16_t Cool_28[] = {9013 ,-4465 ,  678 ,-1630 ,  680 , -554 ,  653 , -553 ,  654 ,-1627 ,  683 ,-1625 ,  674 , -558 ,  649 , -558 ,
649 , -557 ,  651 , -556 ,  651 , -555 ,  652 ,-1629 ,  680 ,-1627 ,  683 , -550 ,  657 , -549 ,  648 , -532 ,
675 , -558 ,  649 , -557 ,  650 , -556 ,  652 , -555 ,  652 , -554 ,  653 , -553 ,  654 ,-1627 ,  683 ,-1625 ,
684 , -549 ,  648 , -559 ,  648 , -558 ,  649 , -557 ,  650 , -556 ,  652 ,-1656 ,  674 , -559 ,  648 ,-1634 ,
676 , -557 ,  650 , -556 ,  651 ,-1630 ,  679 , -553 ,  655 ,-9999,-9957 ,  679 , -553 ,  654 ,-1627 ,  683 , -550 ,
657 , -549 ,  648 , -558 ,  649 , -557 ,  650 , -556 ,  651 , -556 ,  651 , -554 ,  654 , -553 ,  654 , -552 ,
655 , -552 ,  655 , -551 ,  657 ,-1624 ,  675 , -558 ,  649 , -557 ,  650 , -556 ,  651 , -555 ,  652 , -555 ,
652 , -553 ,  655 , -552 ,  655 , -550 ,  657 , -549 ,  648 , -559 ,  648 , -559 ,  648 , -558 ,  649 , -557 ,
650 , -556 ,  652 ,-1628 ,  681 , -552 ,  655 , -551 ,  656 , -550 ,  658 }; //AnalysIR Batch Export (IRremote) - RAW
 int16_t Cool_29[] = {9014 ,-4465 ,  678 ,-1629 ,  681 , -552 ,  655 , -551 ,  656 ,-1625 ,  674 ,-1633 ,  676 , -557 ,  651 , -555 ,
652 , -555 ,  652 ,-1628 ,  682 , -552 ,  655 ,-1626 ,  683 ,-1625 ,  674 , -559 ,  648 , -559 ,  649 , -558 ,
649 , -558 ,  649 , -557 ,  650 , -556 ,  651 , -555 ,  653 , -552 ,  655 , -552 ,  655 ,-1626 ,  684 ,-1624 ,
675 , -558 ,  649 , -557 ,  650 , -557 ,  650 , -555 ,  652 , -554 ,  653 ,-1628 ,  682 , -553 ,  654 ,-1627 ,
682 , -551 ,  657 , -549 ,  658 ,-1624 ,  675 , -558 ,  649 ,-9999,-9964 ,  683 , -551 ,  656 ,-1626 ,  684 , -549 ,
647 , -558 ,  650 , -556 ,  651 , -556 ,  651 , -555 ,  652 , -554 ,  653 , -554 ,  654 , -552 ,  655 , -551 ,
656 , -550 ,  647 , -559 ,  648 ,-1632 ,  677 , -556 ,  652 , -555 ,  652 , -555 ,  652 , -554 ,  653 , -553 ,
655 , -551 ,  656 , -550 ,  657 , -549 ,  648 , -559 ,  648 , -558 ,  649 , -557 ,  650 , -557 ,  650 , -556 ,
652 , -555 ,  652 , -555 ,  652 ,-1628 ,  681 , -552 ,  656 , -550 ,  657 }; //AnalysIR Batch Export (IRremote) - RAW
 int16_t Cool_30[] = {9020 ,-4459 ,  674 ,-1633 ,  677 , -556 ,  651 , -555 ,  652 ,-1629 ,  680 ,-1628 ,  682 , -551 ,  656 , -551 ,
656 , -549 ,  648 , -558 ,  649 ,-1632 ,  677 ,-1631 ,  679 ,-1629 ,  680 , -553 ,  654 , -552 ,  656 , -550 ,
657 , -550 ,  646 , -559 ,  649 , -558 ,  649 , -557 ,  650 , -556 ,  651 , -556 ,  651 ,-1629 ,  681 ,-1626 ,
683 , -550 ,  647 , -559 ,  648 , -559 ,  648 , -558 ,  650 , -556 ,  651 ,-1630 ,  679 , -554 ,  653 ,-1628 ,
682 , -552 ,  655 , -551 ,  656 ,-1625 ,  674 , -559 ,  648 ,-9999,-9966 ,  681 , -551 ,  657 ,-1624 ,  675 , -558 ,
649 , -558 ,  649 , -557 ,  651 , -555 ,  652 , -554 ,  653 , -553 ,  654 , -552 ,  655 , -552 ,  656 , -549 ,
647 , -560 ,  648 , -558 ,  649 ,-1632 ,  677 , -556 ,  651 , -555 ,  653 , -527 ,  680 , -553 ,  654 , -551 ,
656 , -551 ,  656 , -550 ,  647 , -559 ,  648 , -558 ,  650 , -556 ,  651 , -555 ,  652 , -554 ,  653 , -554 ,
654 , -551 ,  656 ,-1626 ,  683 ,-1624 ,  675 , -558 ,  650 , -556 ,  651}; //AnalysIR Batch Export (IRremote) - RAW


 int16_t Heat_16[] = {9044 ,-4461 ,  675 , -557 ,  651 , -556 ,  651 ,-1657 ,  654 ,-1654 ,  656 ,-1651 ,  649 , -558 ,  650 , -556 ,
651 , -556 ,  652 , -554 ,  654 , -552 ,  656 , -550 ,  647 , -559 ,  649 , -557 ,  651 , -556 ,  652 , -528 ,
680 , -552 ,  656 , -550 ,  657 , -550 ,  648 , -558 ,  649 , -557 ,  651 , -554 ,  654 ,-1653 ,  647 ,-1661 ,
649 , -558 ,  650 , -529 ,  679 , -554 ,  654 , -553 ,  655 , -551 ,  657 ,-1651 ,  649 , -557 ,  650 ,-1658 ,
653 , -553 ,  655 , -551 ,  657 ,-1650 ,  650 , -557 ,  650 ,-9999,-9963 ,  684 , -549 ,  648 ,-1634 ,  677 , -555 ,
653 , -554 ,  654 , -552 ,  655 , -551 ,  657 , -550 ,  647 , -559 ,  649 , -557 ,  651 , -555 ,  653 , -553 ,
654 , -553 ,  655 , -551 ,  647 ,-1660 ,  650 , -556 ,  652 , -555 ,  653 , -552 ,  656 , -551 ,  657 , -549 ,
648 , -558 ,  650 , -556 ,  651 , -556 ,  652 , -553 ,  655 , -552 ,  656 , -550 ,  647 , -559 ,  649 , -558 ,
650 , -556 ,  651 , -555 ,  653 , -553 ,  655 , -551 ,  657 ,-1650 ,  650 };
 int16_t Heat_17[] = {9046 ,-4460 ,  675 , -557 ,  651 , -530 ,  678 ,-1655 ,  656 ,-1625 ,  675 ,-1634 ,  676 , -531 ,  677 , -556 ,
652 , -554 ,  654 ,-1654 ,  646 , -560 ,  648 , -558 ,  650 , -531 ,  677 , -555 ,  653 , -553 ,  655 , -551 ,
646 , -559 ,  649 , -558 ,  650 , -556 ,  652 , -554 ,  654 , -552 ,  655 , -552 ,  656 ,-1651 ,  649 ,-1659 ,
652 , -555 ,  653 , -527 ,  681 , -551 ,  657 , -550 ,  657 , -550 ,  648 ,-1634 ,  677 , -556 ,  651 ,-1656 ,
655 , -551 ,  657 , -549 ,  648 ,-1660 ,  651 , -555 ,  653 ,-9999,-9962 ,  674 , -559 ,  649 ,-1632 ,  679 , -553 ,
654 , -553 ,  655 , -551 ,  657 , -549 ,  648 , -559 ,  649 , -557 ,  651 , -555 ,  653 , -553 ,  654 , -552 ,
656 , -550 ,  648 , -558 ,  649 ,-1633 ,  678 , -555 ,  653 , -552 ,  656 , -550 ,  647 , -559 ,  649 , -557 ,
651 , -556 ,  652 , -555 ,  653 , -553 ,  655 , -550 ,  647 , -560 ,  648 , -558 ,  649 , -557 ,  651 , -555 ,
653 , -553 ,  656 ,-1625 ,  674 , -559 ,  649 , -557 ,  651 ,-1657 ,  654};
 int16_t Heat_18[] = {9046 ,-4459 ,  677 , -555 ,  653 , -554 ,  654 ,-1654 ,  656 ,-1626 ,  675 ,-1659 ,  651 , -530 ,  678 , -554 ,
654 , -552 ,  656 , -551 ,  657 ,-1650 ,  650 , -557 ,  650 , -529 ,  679 , -553 ,  655 , -552 ,  656 , -549 ,
648 , -559 ,  649 , -558 ,  650 , -555 ,  653 , -553 ,  655 , -551 ,  656 , -550 ,  648 ,-1633 ,  677 ,-1658 ,
653 , -554 ,  654 , -552 ,  656 , -550 ,  647 , -559 ,  649 , -557 ,  651 ,-1656 ,  654 , -552 ,  656 ,-1651 ,
649 , -558 ,  650 , -530 ,  678 ,-1656 ,  654 , -553 ,  655 ,-9999,-9960 ,  761 , -445 ,  679 ,-1628 ,  682 , -553 ,
645 , -559 ,  649 , -557 ,  651 , -555 ,  652 , -554 ,  654 , -553 ,  655 , -551 ,  647 , -559 ,  648 , -559 ,
649 , -556 ,  652 , -554 ,  654 ,-1655 ,  656 , -550 ,  647 , -560 ,  648 , -558 ,  650 , -556 ,  651 , -557 ,
651 , -555 ,  653 , -553 ,  655 , -551 ,  647 , -559 ,  648 , -559 ,  649 , -557 ,  651 , -555 ,  653 , -554 ,
654 , -551 ,  657 , -549 ,  648 ,-1660 ,  651 , -555 ,  653 ,-1655 ,  655};
 int16_t Heat_19[] = {9024 ,-4456 ,  679 , -554 ,  654 , -553 ,  655 ,-1625 ,  676 ,-1659 ,  651 ,-1630 ,  680 , -552 ,  656 , -551 ,
657 , -549 ,  649 ,-1658 ,  652 ,-1655 ,  656 , -552 ,  656 , -550 ,  647 , -558 ,  650 , -556 ,  651 , -556 ,
652 , -555 ,  653 , -553 ,  655 , -551 ,  657 , -549 ,  648 , -558 ,  650 , -557 ,  651 ,-1656 ,  654 ,-1654 ,
647 , -559 ,  648 , -558 ,  650 , -556 ,  652 , -554 ,  654 , -552 ,  656 ,-1626 ,  674 , -559 ,  649 ,-1632 ,
679 , -554 ,  653 , -553 ,  655 ,-1653 ,  647 , -559 ,  649 ,-9999,-9965 ,  682 , -551 ,  646 ,-1635 ,  676 , -557 ,
651 , -554 ,  654 , -552 ,  655 , -552 ,  656 , -550 ,  648 , -559 ,  649 , -557 ,  650 , -556 ,  652 , -555 ,
653 , -553 ,  655 , -551 ,  657 ,-1624 ,  676 , -557 ,  651 , -556 ,  652 , -554 ,  653 , -554 ,  654 , -552 ,
656 , -550 ,  648 , -559 ,  649 , -557 ,  650 , -557 ,  651 , -559 ,  649 , -553 ,  655 , -551 ,  657 , -549 ,
648 , -558 ,  650 ,-1657 ,  654 ,-1654 ,  656 , -550 ,  648 ,-1660 ,  650 };
 int16_t Heat_20[] = {9045 ,-4458 ,  677 , -529 ,  679 , -554 ,  654 ,-1653 ,  648 ,-1660 ,  650 ,-1657 ,  654 , -553 ,  655 , -551 ,
657 , -549 ,  648 , -558 ,  650 , -556 ,  652 ,-1655 ,  656 , -552 ,  655 , -550 ,  648 , -559 ,  648 , -558 ,
650 , -556 ,  652 , -554 ,  654 , -552 ,  656 , -550 ,  648 , -559 ,  648 , -558 ,  650 ,-1631 ,  680 ,-1654 ,
657 , -550 ,  657 , -523 ,  675 , -558 ,  650 , -556 ,  652 , -554 ,  654 ,-1628 ,  682 , -551 ,  657 ,-1651 ,
649 , -557 ,  651 , -556 ,  652 ,-1655 ,  655 , -551 ,  647 ,-9999,-9967 ,  680 , -527 ,  681 ,-1653 ,  658 , -548 ,
649 , -531 ,  677 , -556 ,  652 , -554 ,  654 , -553 ,  655 , -550 ,  647 , -559 ,  649 , -557 ,  651 , -556 ,
652 , -554 ,  654 , -553 ,  655 ,-1652 ,  648 , -559 ,  649 , -530 ,  678 , -555 ,  653 , -553 ,  654 , -552 ,
656 , -550 ,  648 , -559 ,  649 , -556 ,  651 , -555 ,  653 , -554 ,  654 , -552 ,  656 , -550 ,  647 , -560 ,
648 , -557 ,  651 , -555 ,  653 , -554 ,  654 ,-1653 ,  658 ,-1649 ,  651};
 int16_t Heat_21[] = {9043 ,-4463 ,  683 , -523 ,  674 , -533 ,  675 ,-1632 ,  679 ,-1656 ,  654 ,-1626 ,  675 , -532 ,  675 , -531 ,
677 , -556 ,  652 ,-1629 ,  682 , -551 ,  657 ,-1623 ,  677 , -556 ,  651 , -556 ,  652 , -554 ,  654 , -526 ,
682 , -524 ,  683 , -550 ,  648 , -558 ,  649 , -557 ,  651 , -555 ,  653 , -553 ,  655 ,-1626 ,  674 ,-1660 ,
651 , -555 ,  652 , -555 ,  653 , -526 ,  682 , -550 ,  658 , -549 ,  648 ,-1660 ,  651 , -529 ,  679 ,-1654 ,
656 , -551 ,  657 , -523 ,  675 ,-1659 ,  651 , -554 ,  654 ,-9999,-9959 ,  677 , -555 ,  653 ,-1629 ,  682 , -524 ,
683 , -550 ,  648 , -557 ,  651 , -556 ,  651 , -555 ,  653 , -553 ,  655 , -552 ,  656 , -550 ,  647 , -559 ,
649 , -557 ,  651 , -556 ,  652 ,-1655 ,  655 , -551 ,  657 , -549 ,  648 , -558 ,  650 , -557 ,  650 , -556 ,
652 , -554 ,  654 , -552 ,  656 , -550 ,  647 , -559 ,  649 , -557 ,  651 , -529 ,  679 , -554 ,  654 , -552 ,
656 , -549 ,  648 ,-1660 ,  651 , -555 ,  653 ,-1655 ,  655 ,-1626 ,  674};
 int16_t Heat_22[] = {9013 ,-4467 ,  679 , -553 ,  655 , -552 ,  656 ,-1625 ,  675 ,-1632 ,  679 ,-1629 ,  682 , -524 ,  684 , -549 ,
648 , -559 ,  649 , -556 ,  652 ,-1655 ,  655 ,-1652 ,  649 , -559 ,  648 , -558 ,  650 , -556 ,  652 , -555 ,
653 , -553 ,  655 , -551 ,  656 , -550 ,  648 , -558 ,  650 , -556 ,  651 , -556 ,  652 ,-1655 ,  656 ,-1625 ,
675 , -558 ,  650 , -556 ,  652 , -554 ,  654 , -552 ,  656 , -551 ,  646 ,-1661 ,  650 , -556 ,  651 ,-1656 ,
655 , -552 ,  656 , -550 ,  647 ,-1661 ,  650 , -556 ,  652 ,-9999,-9963 ,  673 , -533 ,  675 ,-1633 ,  678 , -555 ,
653 , -553 ,  655 , -525 ,  683 , -549 ,  648 , -558 ,  650 , -556 ,  651 , -556 ,  652 , -554 ,  654 , -552 ,
656 , -551 ,  657 , -549 ,  649 ,-1660 ,  650 , -556 ,  652 , -555 ,  653 , -553 ,  655 , -552 ,  656 , -550 ,
647 , -559 ,  649 , -557 ,  651 , -556 ,  652 , -553 ,  655 , -551 ,  656 , -551 ,  647 , -559 ,  649 , -558 ,
650 , -556 ,  651 , -555 ,  653 ,-1628 ,  683 ,-1652 ,  648 ,-1634 ,  676};
 int16_t Heat_23[] = {9040 ,-4464 ,  682 , -551 ,  657 , -549 ,  649 ,-1632 ,  679 ,-1655 ,  655 ,-1627 ,  674 , -532 ,  676 , -557 ,
650 , -556 ,  652 ,-1655 ,  656 ,-1652 ,  648 ,-1660 ,  651 , -555 ,  653 , -554 ,  654 , -552 ,  656 , -550 ,
647 , -559 ,  649 , -558 ,  650 , -556 ,  652 , -555 ,  653 , -553 ,  654 , -552 ,  656 ,-1652 ,  649 ,-1633 ,
677 , -556 ,  652 , -554 ,  654 , -553 ,  655 , -551 ,  657 , -550 ,  648 ,-1660 ,  650 , -530 ,  678 ,-1656 ,
654 , -553 ,  655 , -551 ,  657 ,-1651 ,  649 , -557 ,  651 ,-9999,-9962 ,  675 , -557 ,  651 ,-1657 ,  654 , -526 ,
682 , -552 ,  655 , -551 ,  647 , -559 ,  649 , -557 ,  651 , -556 ,  651 , -555 ,  653 , -553 ,  655 , -552 ,
656 , -550 ,  648 , -558 ,  649 ,-1658 ,  653 , -553 ,  655 , -552 ,  656 , -551 ,  646 , -560 ,  648 , -558 ,
650 , -557 ,  651 , -556 ,  652 , -554 ,  654 , -553 ,  654 , -552 ,  656 , -551 ,  647 , -559 ,  649 , -557 ,
651 , -555 ,  653 ,-1655 ,  655 ,-1626 ,  675 ,-1632 ,  678 ,-1630 ,  681 };
 int16_t Heat_24[] = {9016 ,-4464 ,  682 , -551 ,  657 , -550 ,  648 ,-1633 ,  677 ,-1630 ,  681 ,-1627 ,  683 , -550 ,  648 , -559 ,
649 , -557 ,  650 , -529 ,  679 , -554 ,  654 , -552 ,  656 ,-1652 ,  649 , -557 ,  650 , -557 ,  651 , -555 ,
653 , -553 ,  655 , -552 ,  656 , -550 ,  647 , -559 ,  649 , -557 ,  651 , -555 ,  653 ,-1654 ,  656 ,-1652 ,
649 , -558 ,  650 , -556 ,  651 , -555 ,  653 , -554 ,  654 , -552 ,  656 ,-1652 ,  648 , -558 ,  650 ,-1657 ,
654 , -553 ,  655 , -551 ,  657 ,-1652 ,  648 , -558 ,  650 ,-9999,-9963 ,  674 , -559 ,  649 ,-1633 ,  678 , -555 ,
653 , -554 ,  654 , -552 ,  656 , -550 ,  648 , -558 ,  649 , -558 ,  650 , -556 ,  652 , -554 ,  654 , -553 ,
655 , -551 ,  647 , -559 ,  648 ,-1633 ,  678 , -555 ,  653 , -553 ,  655 , -551 ,  647 , -559 ,  648 , -558 ,
650 , -556 ,  652 , -554 ,  654 , -553 ,  655 , -551 ,  657 , -549 ,  648 , -558 ,  650 , -556 ,  652 , -555 ,
653 , -553 ,  655 , -551 ,  657 , -549 ,  648 , -559 ,  649 , -557 ,  651};
 int16_t Heat_25[] = {9043 ,-4460 ,  683 , -551 ,  646 , -559 ,  650 ,-1658 ,  652 ,-1629 ,  682 , -551 ,  657 ,-1650 ,  650 , -557 ,
651 , -555 ,  653 ,-1654 ,  657 , -550 ,  648 , -558 ,  650 ,-1657 ,  653 , -554 ,  654 , -553 ,  655 , -551 ,
647 , -559 ,  649 , -558 ,  649 , -557 ,  652 , -554 ,  654 , -553 ,  655 , -551 ,  656 ,-1652 ,  649 ,-1633 ,
677 , -555 ,  653 , -554 ,  654 , -552 ,  656 , -550 ,  648 , -559 ,  649 ,-1658 ,  652 , -555 ,  653 ,-1654 ,
657 , -550 ,  648 , -559 ,  648 ,-1659 ,  652 , -555 ,  653 ,-9999,-9960 ,  679 , -554 ,  654 ,-1627 ,  684 , -549 ,
648 , -559 ,  649 , -557 ,  651 , -555 ,  653 , -553 ,  655 , -551 ,  657 , -549 ,  649 , -557 ,  650 , -556 ,
652 , -554 ,  654 , -553 ,  655 ,-1652 ,  649 , -558 ,  649 , -557 ,  651 , -555 ,  653 , -554 ,  654 , -552 ,
656 , -550 ,  658 , -549 ,  648 , -559 ,  649 , -558 ,  650 , -556 ,  652 , -554 ,  654 , -552 ,  656 , -550 ,
647 , -559 ,  649 ,-1659 ,  652 , -554 ,  654 , -553 ,  655 , -550 ,  647};
 int16_t Heat_26[] = {9012 ,-4467 ,  680 , -553 ,  655 , -552 ,  656 ,-1625 ,  675 ,-1660 ,  651 , -556 ,  652 ,-1629 ,  682 , -551 ,
656 , -550 ,  648 , -559 ,  649 ,-1659 ,  651 , -556 ,  652 ,-1628 ,  683 , -550 ,  648 , -558 ,  649 , -558 ,
650 , -557 ,  651 , -554 ,  654 , -553 ,  655 , -551 ,  646 , -560 ,  648 , -558 ,  650 ,-1657 ,  654 ,-1627 ,
683 , -551 ,  647 , -559 ,  649 , -558 ,  650 , -556 ,  652 , -555 ,  653 ,-1655 ,  656 , -551 ,  656 ,-1652 ,
649 , -558 ,  650 , -556 ,  652 ,-1656 ,  654 , -553 ,  655 ,-9999,-9959 ,  679 , -554 ,  654 ,-1627 ,  684 , -549 ,
648 , -559 ,  649 , -558 ,  650 , -556 ,  652 , -554 ,  654 , -552 ,  656 , -550 ,  647 , -559 ,  649 , -557 ,
651 , -555 ,  653 , -554 ,  654 ,-1627 ,  683 , -550 ,  648 , -558 ,  650 , -556 ,  652 , -554 ,  653 , -554 ,
654 , -552 ,  656 , -550 ,  648 , -559 ,  649 , -557 ,  650 , -556 ,  652 , -554 ,  654 , -552 ,  656 , -550 ,
647 , -560 ,  648 , -558 ,  650 ,-1631 ,  680 , -552 ,  656 , -550 ,  647};
 int16_t Heat_27[] = {9048 ,-4459 ,  677 , -556 ,  652 , -554 ,  654 ,-1627 ,  673 ,-1634 ,  677 , -557 ,  651 ,-1656 ,  655 , -552 ,
656 , -549 ,  649 ,-1659 ,  651 ,-1630 ,  681 , -552 ,  656 ,-1625 ,  675 , -559 ,  649 , -557 ,  651 , -555 ,
653 , -553 ,  655 , -551 ,  656 , -550 ,  648 , -558 ,  650 , -556 ,  652 , -554 ,  653 ,-1654 ,  657 ,-1625 ,
675 , -557 ,  651 , -556 ,  652 , -555 ,  653 , -553 ,  655 , -552 ,  656 ,-1651 ,  649 , -557 ,  651 ,-1631 ,
680 , -553 ,  655 , -552 ,  656 ,-1652 ,  648 , -558 ,  650 ,-9999,-9962 ,  676 , -557 ,  652 ,-1629 ,  681 , -552 ,
656 , -551 ,  657 , -550 ,  647 , -560 ,  648 , -558 ,  650 , -556 ,  652 , -555 ,  653 , -552 ,  656 , -551 ,
657 , -549 ,  648 , -559 ,  649 ,-1659 ,  652 , -554 ,  654 , -552 ,  656 , -550 ,  648 , -558 ,  650 , -556 ,
652 , -554 ,  654 , -552 ,  655 , -551 ,  658 , -549 ,  648 , -558 ,  650 , -556 ,  652 , -554 ,  654 , -552 ,
656 , -551 ,  657 ,-1624 ,  676 ,-1632 ,  679 , -554 ,  654 , -552 ,  656};
 int16_t Heat_28[] = {9047 ,-4459 ,  677 , -555 ,  653 , -554 ,  654 ,-1627 ,  684 ,-1624 ,  676 ,-1632 ,  679 ,-1629 ,  681 , -552 ,
656 , -551 ,  647 , -560 ,  648 , -559 ,  649 ,-1632 ,  679 ,-1629 ,  681 , -552 ,  656 , -550 ,  648 , -558 ,
649 , -558 ,  650 , -556 ,  652 , -554 ,  654 , -552 ,  656 , -550 ,  647 , -559 ,  649 ,-1633 ,  678 ,-1629 ,
682 , -550 ,  658 , -549 ,  648 , -559 ,  649 , -557 ,  651 , -555 ,  653 ,-1654 ,  657 , -549 ,  648 ,-1660 ,
651 , -556 ,  651 , -556 ,  652 ,-1629 ,  682 , -552 ,  656 ,-9999,-9957 ,  682 , -551 ,  656 ,-1652 ,  649 , -558 ,
650 , -556 ,  651 , -556 ,  652 , -554 ,  654 , -552 ,  656 , -551 ,  647 , -559 ,  649 , -557 ,  651 , -556 ,
652 , -554 ,  654 , -553 ,  655 ,-1625 ,  675 , -559 ,  649 , -558 ,  650 , -556 ,  652 , -554 ,  654 , -553 ,
654 , -551 ,  657 , -550 ,  648 , -558 ,  649 , -557 ,  651 , -556 ,  652 , -554 ,  654 , -551 ,  657 , -550 ,
648 , -559 ,  649 , -557 ,  650 , -556 ,  652 ,-1629 ,  682 , -552 ,  656};
 int16_t Heat_29[] = {9044 ,-4462 ,  674 , -559 ,  649 , -557 ,  651 ,-1630 ,  681 ,-1627 ,  684 ,-1624 ,  676 ,-1631 ,  680 , -553 ,
655 , -552 ,  656 ,-1625 ,  675 , -558 ,  650 ,-1657 ,  654 ,-1628 ,  683 , -549 ,  648 , -558 ,  650 , -557 ,
651 , -556 ,  652 , -554 ,  654 , -553 ,  655 , -551 ,  657 , -550 ,  647 , -559 ,  649 ,-1632 ,  679 ,-1628 ,
683 , -551 ,  646 , -560 ,  648 , -558 ,  650 , -556 ,  652 , -554 ,  654 ,-1627 ,  674 , -559 ,  649 ,-1658 ,
653 , -553 ,  655 , -552 ,  656 ,-1626 ,  674 , -558 ,  650 ,-9999,-9964 ,  675 , -558 ,  650 ,-1657 ,  653 , -554 ,
654 , -553 ,  655 , -551 ,  647 , -559 ,  649 , -558 ,  650 , -556 ,  652 , -553 ,  655 , -551 ,  657 , -550 ,
647 , -559 ,  649 , -557 ,  651 ,-1630 ,  681 , -551 ,  657 , -550 ,  647 , -559 ,  649 , -558 ,  650 , -556 ,
652 , -554 ,  654 , -552 ,  656 , -550 ,  648 , -558 ,  649 , -557 ,  651 , -555 ,  653 , -553 ,  655 , -552 ,
656 , -551 ,  646 ,-1635 ,  676 , -556 ,  652 ,-1656 ,  655 , -551 ,  657};
 int16_t Heat_30[] = {9019 ,-4461 ,  675 , -558 ,  650 , -557 ,  651 ,-1656 ,  655 ,-1626 ,  675 ,-1633 ,  678 ,-1630 ,  681 , -552 ,
656 , -551 ,  657 , -549 ,  648 ,-1631 ,  680 ,-1628 ,  683 ,-1626 ,  674 , -558 ,  650 , -557 ,  651 , -556 ,
652 , -554 ,  654 , -553 ,  655 , -551 ,  647 , -558 ,  650 , -557 ,  650 , -557 ,  651 ,-1656 ,  655 ,-1627 ,
684 , -552 ,  646 , -558 ,  650 , -557 ,  651 , -555 ,  653 , -553 ,  655 ,-1625 ,  675 , -558 ,  650 ,-1657 ,
654 , -553 ,  655 , -551 ,  657 ,-1625 ,  675 , -558 ,  650 ,-9999,-9964 ,  675 , -557 ,  651 ,-1656 ,  655 , -552 ,
656 , -551 ,  647 , -559 ,  649 , -557 ,  651 , -555 ,  653 , -555 ,  652 , -553 ,  655 , -551 ,  647 , -559 ,
649 , -557 ,  651 , -555 ,  653 ,-1629 ,  682 , -551 ,  657 , -549 ,  648 , -559 ,  649 , -557 ,  651 , -556 ,
652 , -554 ,  654 , -553 ,  655 , -551 ,  647 , -559 ,  649 , -558 ,  650 , -556 ,  652 , -554 ,  654 , -552 ,
656 , -550 ,  647 , -560 ,  648 ,-1633 ,  678 ,-1631 ,  680 , -553 ,  655};
 

 int16_t Off[] = {9016 ,-4464 ,  678 ,-1629 ,  680 , -527 ,  679 , -527 ,  680 , -526 ,  681 ,-1626 ,  683 , -524 ,  683 , -523 ,
684 , -522 ,  674 , -532 ,  675 ,-1632 ,  676 ,-1631 ,  678 ,-1631 ,  678 , -529 ,  678 , -528 ,  679 , -527 ,
679 , -527 ,  680 , -527 ,  680 , -552 ,  655 , -525 ,  682 , -524 ,  683 , -523 ,  683 ,-1624 ,  675 , -557 ,
650 , -530 ,  676 , -531 ,  676 , -557 ,  650 , -529 ,  678 , -529 ,  678 ,-1628 ,  680 , -527 ,  680 ,-1627 ,
681 , -526 ,  681 , -525 ,  682 ,-1625 ,  684 , -523 ,  684 ,-9999,-9956 ,  684 , -523 ,  674 ,-1633 ,  675 , -533 ,
674 , -532 ,  675 , -531 ,  676 , -530 ,  677 , -556 ,  650 , -529 ,  678 , -529 ,  678 , -527 ,  680 , -526 ,
681 , -526 ,  680 , -552 ,  655 ,-1627 ,  682 , -524 ,  683 , -524 ,  682 , -525 ,  682 , -550 ,  657 , -524 ,
683 , -523 ,  684 , -523 ,  673 , -533 ,  674 , -532 ,  674 , -532 ,  675 , -532 ,  675 , -530 ,  677 , -530 ,
677 , -529 ,  677 ,-1631 ,  678 ,-1630 ,  679 , -527 ,  679 ,-1629 ,  680}; //AnalysIR Batch Export (IRremote) - RAW
 
 


 void ac_button_off(){

	ir_tx_init();
	ir_raw_send(Off, sizeof(Off) / sizeof(*Off));

}

 void ac_button_aut(){

	ir_tx_init();
	ir_raw_send(Fan, sizeof(Fan) / sizeof(*Fan));

}

void ac_command(int target_state, float target_temp){

int16_t *RawData;
uint16_t length1;

if(target_state == 2)
{
//Cool mode

		if (target_temp ==  16.0)
		{
		RawData = Cool_16;
		length1 = sizeof(Cool_16)/ sizeof(*Cool_16);
		}
		else if (target_temp ==  17.0)
		{
		RawData = Cool_17;
		length1 =sizeof(Cool_17)/ sizeof(*Cool_17);
		}
		else if (target_temp ==  18.0)
		{
			
		RawData = Cool_18;
		length1 = sizeof(Cool_18)/sizeof(*Cool_18);
		}
		else if (target_temp ==  19.0)
		{
		RawData = Cool_19;
		length1 = sizeof(Cool_19)/sizeof(*Cool_19);
		}
		else if (target_temp ==  20.0)
		{
		RawData = Cool_20;
		length1 = sizeof(Cool_20)/sizeof(*Cool_20);
		}
		else if (target_temp ==  21.0)
		{
		RawData = Cool_21;
		length1 = sizeof(Cool_21)/sizeof(*Cool_21);
		}
		else if (target_temp ==  22.0)
		{
		RawData = Cool_22;
		length1 =sizeof(Cool_22)/ sizeof(*Cool_22);
		}
		else if (target_temp ==  23.0)
		{
		RawData = Cool_23;
		length1 = sizeof(Cool_23)/sizeof(*Cool_23);
		}
		else if (target_temp ==  24.0)
		{
		RawData = Cool_24;
		length1 = sizeof(Cool_24)/sizeof(*Cool_24);
		}
		else if (target_temp ==  25.0)
		{
		RawData = Cool_25;
		length1 = sizeof(Cool_25)/sizeof(*Cool_25);
		}
		else if (target_temp ==  26.0)
		{
		RawData = Cool_26;
		length1 = sizeof(Cool_26)/sizeof(*Cool_26);
		}
		else if (target_temp ==  27.0)
		{
		RawData = Cool_27;
		length1 = sizeof(Cool_27)/sizeof(*Cool_27);
		}
		else if (target_temp ==  28.0)
		{
		RawData = Cool_28;
		length1 =sizeof(Cool_28)/ sizeof(*Cool_28);
		}
		else if (target_temp ==  29.0)
		{	
		RawData = Cool_29;
		length1 = sizeof(Cool_29)/sizeof(*Cool_29);
		}
		else if (target_temp ==  30.0)
		{	
		RawData = Cool_30;
		length1 = sizeof(Cool_30)/sizeof(*Cool_30);
		} 
		
	}
	
	if( target_state == 1)
	{
		//Heat mode
		if (target_temp ==  16.0)
		{
		RawData = Heat_16;
		length1 = sizeof(Heat_16)/sizeof(*Heat_16);
		}
		else if (target_temp ==  17.0)
		{
		RawData = Heat_17;
		length1 = sizeof(Heat_17)/sizeof(*Heat_17);
		}
		else if (target_temp ==  18.0)
		{
		RawData = Heat_18;
		length1 = sizeof(Heat_18)/sizeof(*Heat_18);
		}
		else if (target_temp ==  19.0)
		{
		RawData = Heat_19;
		length1 = sizeof(Heat_19)/sizeof(*Heat_19);
		}
		else if (target_temp ==  20.0)
		{
		RawData = Heat_20;
		length1 = sizeof(Heat_20)/sizeof(*Heat_20);
		}
		else if (target_temp ==  21.0)
		{
		RawData = Heat_21;
		length1 = sizeof(Heat_21)/sizeof(*Heat_21);
		}
		else if (target_temp ==  22.0)
		{
		RawData = Heat_22;
		length1 = sizeof(Heat_22)/sizeof(*Heat_22);
		}
		else if (target_temp ==  23.0)
		{
		RawData = Heat_23;
		length1 = sizeof(Heat_23)/sizeof(*Heat_23);
		}
		else if (target_temp ==  24.0)
		{
		RawData = Heat_24;
		length1 = sizeof(Heat_24)/sizeof(*Heat_24);
		}
		else if (target_temp ==  25.0)
		{
		RawData = Heat_25;
		length1 = sizeof(Heat_25)/sizeof(*Heat_25);
		}
		else if (target_temp ==  26.0)
		{
		RawData = Heat_26;
		length1 = sizeof(Heat_26)/sizeof(*Heat_26);
		}
		else if (target_temp ==  27.0)
		{
		RawData = Heat_27;
		length1 = sizeof(Heat_27)/sizeof(*Heat_27);
		}
		else if (target_temp ==  28.0)
		{
		RawData = Heat_28;
		length1 = sizeof(Heat_28)/sizeof(*Heat_28);
		}
		else if (target_temp ==  29.0)
		{	
		RawData = Heat_29;
		length1 = sizeof(Heat_29)/sizeof(*Heat_29);
		}
		else if (target_temp ==  30.0)
		{	
		RawData = Heat_30;
		length1 = sizeof(Heat_30)/sizeof(*Heat_30);
		} 

	}
	
	if( target_state == 0)
	{
		//printf("Fan mode\n", 30 );
		RawData = Off;
		length1 = sizeof(Off)/sizeof(*Off);
	}

	ir_tx_init();
	ir_raw_send(RawData, length1);


}