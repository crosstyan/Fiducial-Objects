#ifndef _COLORS_
#define _COLORS_

// Web:
// https://www.perpetualpc.net/6429_colors.html#color_list



/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Personalized colouring
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
// Personal colour palettese
    // Primary = {255, 8, 90}
        #define Colour_Primary "\033[38;2;"<<"255"<<";"<<"8"<<";"<<"90"<<"m"
        #define Colour_Primary_Bold "\033[1;38;2;"<<"255"<<";"<<"8"<<";"<<"90"<<"m"
        #define Colour_Primary_Underline "\033[4;38;2;"<<"255"<<";"<<"8"<<";"<<"90"<<"m"
        #define Colour_Primary_Background "\033[48;2;"<<"255"<<";"<<"8"<<";"<<"90"<<"m"
        #define Colour_Primary_Bold_Underline "\033[1;4;38;2;"<<"255"<<";"<<"8"<<";"<<"90"<<"m"
        #define Colour_Primary_Bold_Background "\033[1;48;2;"<<"255"<<";"<<"8"<<";"<<"90"<<"m"
    // Secondary = {226, 79, 255}
        #define Colour_Secondary "\033[38;2;"<<"226"<<";"<<"79"<<";"<<"255"<<"m"
        #define Colour_Secondary_Bold "\033[1;38;2;"<<"226"<<";"<<"79"<<";"<<"255"<<"m"
        #define Colour_Secondary_Underline "\033[4;38;2;"<<"226"<<";"<<"79"<<";"<<"255"<<"m"
        #define Colour_Secondary_Background "\033[48;2;"<<"226"<<";"<<"79"<<";"<<"255"<<"m"
        #define Colour_Secondary_Bold_Underline "\033[1;4;38;2;"<<"226"<<";"<<"79"<<";"<<"255"<<"m"
        #define Colour_Secondary_Bold_Background "\033[1;48;2;"<<"226"<<";"<<"79"<<";"<<"255"<<"m"
    // Tertiary = {123, 79, 255}
        #define Colour_Terciary "\033[38;2;"<<"123"<<";"<<"79"<<";"<<"255"<<"m"
        #define Colour_Terciary_Bold "\033[1;38;2;"<<"123"<<";"<<"79"<<";"<<"255"<<"m"
        #define Colour_Terciary_Underline "\033[4;38;2;"<<"123"<<";"<<"79"<<";"<<"255"<<"m"
        #define Colour_Terciary_Background "\033[48;2;"<<"123"<<";"<<"79"<<";"<<"255"<<"m"
        #define Colour_Terciary_Bold_Underline "\033[1;4;38;2;"<<"123"<<";"<<"79"<<";"<<"255"<<"m"
        #define Colour_Terciary_Bold_Background "\033[1;48;2;"<<"123"<<";"<<"79"<<";"<<"255"<<"m"
    // Quaternary = {79, 138, 255}
        #define Colour_Quaternary "\033[38;2;"<<"79"<<";"<<"138"<<";"<<"255"<<"m"
        #define Colour_Quaternary_Bold "\033[1;38;2;"<<"79"<<";"<<"138"<<";"<<"255"<<"m"
        #define Colour_Quaternary_Underline "\033[4;38;2;"<<"79"<<";"<<"138"<<";"<<"255"<<"m"
        #define Colour_Quaternary_Background "\033[48;2;"<<"79"<<";"<<"138"<<";"<<"255"<<"m"
        #define Colour_Quaternary_Bold_Underline "\033[1;4;38;2;"<<"79"<<";"<<"138"<<";"<<"255"<<"m"
        #define Colour_Quaternary_Bold_Background "\033[1;48;2;"<<"79"<<";"<<"138"<<";"<<"255"<<"m"



/*~~~~~~~~~~~~~~
    STYLES
~~~~~~~~~~~~~~*/
// Bold
#define BOLD "\033[1m"
// Underline
#define UNDERLINE "\033[4m"


/*~~~~~~~~~~~~~~
    COLORS
~~~~~~~~~~~~~~*/
// Standard colors
    #define BLACK  "\x1B[30m"
    #define RED  "\x1B[31m"
    #define GREEN  "\x1B[32m"
    #define YELLOW  "\x1B[33m"
    #define BLUE  "\x1B[34m"
    #define MAGENTA  "\x1B[35m"
    #define CYAN  "\x1B[36m"
    #define WHITE  "\x1B[37m"
// Bold colors
    #define BBLACK  "\033[1m\033[30m"
    #define BRED  "\033[1m\033[31m"
    #define BGREEN  "\033[1m\033[32m"
    #define BYELLOW  "\033[1m\033[33m"
    #define BBLUE  "\033[1m\033[34m"
    #define BMAGENTA  "\033[1m\033[35m"
    #define BCYAN  "\033[1m\033[36m"
    #define BWHITE  "\033[1m\033[37m"
// Underline colors
    #define UBLACK  "\033[4m\033[30m"
    #define URED  "\033[4m\033[31m"
    #define UGREEN  "\033[4m\033[32m"
    #define UYELLOW  "\033[4m\033[33m"
    #define UBLUE  "\033[4m\033[34m"
    #define UMAGENTA  "\033[4m\033[35m"
    #define UCYAN  "\033[4m\033[36m"
    #define UWHITE  "\033[4m\033[37m"
// Background color
    #define BKBLACK  "\x1B[40m"
    #define BKRED  "\x1B[41m"
    #define BKGREEN  "\x1B[42m"
    #define BKYELLOW  "\x1B[43m"
    #define BKBLUE  "\x1B[44m"
    #define BKMAGENTA  "\x1B[45m"
    #define BKCYAN  "\x1B[46m"
    #define BKWHITE  "\x1B[47m"
// Bold background color
    #define BBKBLACK  "\033[1m\033[40m"
    #define BBKRED  "\033[1m\033[41m"
    #define BBKGREEN  "\033[1m\033[42m"
    #define BBKYELLOW  "\033[1m\033[43m"
    #define BBKBLUE  "\033[1m\033[44m"
    #define BBKMAGENTA  "\033[1m\033[45m"
    #define BBKCYAN  "\033[1m\033[46m"
    #define BBKWHITE  "\033[1m\033[47m"
// Commands
    #define RESET  "\x1B[0m"




/*~~~~~~~~~~~~~~~
    SYMBOLS
~~~~~~~~~~~~~~~*/
// Check and Cross symbols:
    #define Symbols_Check "\u2713"
    #define Symbols_Cross "\u2717"
// Arrow symbols:
    #define Symbols_ArrowRight "\u2192"
    #define Symbols_ArrowLeft "\u2190"
    #define Symbols_ArrowUp "\u2191"
    #define Symbols_ArrowDown "\u2193"
// Shape symbols:
    #define Symbols_Diamond "\u2666"
    #define Symbols_Heart "\u2665"
    #define Symbols_Club "\u2663"
    #define Symbols_Spade "\u2660"
    #define Symbols_Square "\u25A0"
    #define Symbols_Circle "\u25CF"
    #define Symbols_Triangle "\u25B2"
// Checkbox symbols:
    #define Symbols_CheckboxChecked "\u2611"
    #define Symbols_CheckboxUnchecked "\u2610"
// Line symbols:
    #define Symbols_Parallel "\u2225"
    #define Symbols_Perpendicular "\u22A5"
// Infinity symbols:
    #define Symbols_Infinity "\u221E"
// Mathematics symbols:
    #define Symbols_PlusMinus "\u00B1"
    #define Symbols_MinusPlus "\u2213"
    #define Symbols_Divide "\u00F7"
    #define Symbols_Multiply "\u00D7"
    #define Symbols_NotEqual "\u2260"
    #define Symbols_LessThan "\u003C"
    #define Symbols_LessThanOrEqual "\u2264"
    #define Symbols_GreaterThan "\u003E"
    #define Symbols_GreaterThanOrEqual "\u2265"
    #define Symbols_SquareRoot "\u221A"
    #define Symbols_Pi "\u03C0"
    #define Symbols_Delta "\u2206"
    #define Symbols_Sigma "\u03A3"
    #define Symbols_Integral "\u222B"
    #define Symbols_Degree "\u00B0"
    #define Symbols_Micro "\u00B5"
    #define Symbols_Angstrom "\u212B"
// Others
    #define Symbols_SquareEmpty "\u25A1"
    #define Symbols_SquareFilled "\u25A3"


#endif  
