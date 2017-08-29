/***************************************************************************** 
 *                            Test Application                               *
 *                                                                           *
 *                                                                           *
 *                                                                           *
 *    Copyright (c)2012           Toby Opferman                              *
 *****************************************************************************/

#define _CRT_SECURE_NO_DEPRECATE

#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <conio.h>
#include "sudoku.h"


typedef struct _SUDOKU_PUZZLES
{
    ULONG SudokuArray[9*9];
        
} SUDOKU_PUZZLES, *PSUDOKU_PUZZLES;

SUDOKU_PUZZLES g_Puzzles[] =
{

    /*
     * Easy Puzzle
     */
    {0,9,3, 1,0,5, 6,4,0,
    7,0,0, 0,0,0, 0,0,5,
    5,0,1, 2,0,9, 3,0,7,

    2,0,0, 0,0,0, 0,0,3,
    0,3,6, 9,0,7, 5,2,0,
    9,0,0, 0,0,0, 0,0,1,

    3,0,2, 4,0,8, 1,0,9,
    6,0,0, 0,0,0, 0,0,4,
    0,4,7, 3,0,2, 8,5,0},

    {0,0,2, 5,0,7, 1,0,3,
    1,0,7, 0,8,0, 0,0,0,
    4,0,0, 0,0,0, 0,0,2,

    0,4,0, 7,0,0, 0,0,1,
    0,0,8, 6,0,5, 3,0,0,
    3,0,0, 0,0,8, 0,5,0,

    8,0,0, 0,0,0, 0,0,7,
    0,0,0, 0,7,0, 9,0,8,
    9,0,1, 8,0,3, 5,0,0},

    {7,4,0, 0,0,0, 9,0,0,
    3,0,0, 0,0,0, 0,7,6,
    0,0,0, 4,7,2, 0,0,3,

    0,9,0, 8,5,0, 0,2,0,
    0,2,0, 0,0,0, 0,3,0,
    0,7,0, 0,6,3, 0,4,0,

    2,0,0, 5,9,8, 0,0,0,
    9,5,0, 0,0,0, 0,0,8,
    0,0,7, 0,0,0, 0,9,4},

   {5,1,2, 4,0,0, 0,6,0,
    7,0,0, 0,0,8, 5,0,0,
    8,3,0, 5,0,0, 0,0,0,
    
    2,0,0, 0,8,0, 0,0,7,
    0,0,5, 2,0,1, 3,0,0,
    3,0,0, 0,4,0, 0,0,6,
    
    0,0,0, 0,0,4, 0,8,2,
    0,0,7, 6,0,0, 0,0,4,
    0,2,0, 0,0,9, 6,7,5},

    {0,0,1, 8,0,0, 0,9,0,
    0,0,6, 0,0,1, 3,0,0,
    3,0,0, 2,0,0, 0,0,5,

    0,9,0, 0,0,0, 2,0,0,
    2,0,0, 0,9,0, 0,0,1,
    0,0,5, 0,0,0, 0,7,0,

    1,0,0, 0,0,5, 0,0,4,
    0,0,7, 4,0,0, 9,0,0,
    0,3,0, 0,0,8, 1,0,0},

   {2,4,0, 5,0,0, 0,8,1,
    0,0,0, 0,0,3, 0,0,5,
    0,0,0, 7,0,2, 4,0,0,

    3,0,0, 0,5,0, 2,6,0,
    0,9,0, 1,0,8, 0,5,0,
    0,5,8, 0,3,0, 0,0,4,

    0,0,9, 8,0,1, 0,0,0,
    8,0,0, 3,0,0, 0,0,0,
    5,3,0, 0,0,4, 0,1,2},

/*
 * Many Solutions
 */
   {2,4,0, 5,0,0, 0,8,1,
    0,0,0, 0,0,3, 0,0,5,
    0,0,0, 7,0,2, 4,0,0,

    3,0,0, 0,0,0, 2,6,0,
    0,9,0, 1,0,8, 0,5,0,
    0,5,8, 0,3,0, 0,0,4,

    0,0,9, 8,0,1, 0,0,0,
    8,0,0, 3,0,0, 0,0,0,
    5,3,0, 0,0,4, 0,1,2},

    /*
     * Hardest Sudoku Puzzle
     */
  { 8,0,0, 0,0,0, 0,0,0,
    0,0,3, 6,0,0, 0,0,0,
    0,7,0, 0,9,0, 2,0,0,

    0,5,0, 0,0,7, 0,0,0,
    0,0,0, 0,4,5, 7,0,0,
    0,0,0, 1,0,0, 0,3,0,

    0,0,1, 0,0,0, 0,6,8,
    0,0,8, 5,0,0, 0,1,0,
    0,9,0, 0,0,0, 4,0,0},


};

/*******************************************************************************
 * main                                                                        *
 *                                                                             *
 * DESCRIPTION: C Entry Point                                                  *
 *                                                                             *
 * INPUT                                                                       *
 *   Standard Parameters                                                       *
 *                                                                             *
 * OUTPUT                                                                      * 
 *   ErrorCode to OS                                                           *
 *                                                                             *
 *******************************************************************************/
int _cdecl main(int argc, char **argv)
{
    HSUDOKU hSudoku;
    UINT Index = 7;

    printf("Solving with Priority Interrupts\n");
    for(Index = 0; Index < sizeof(g_Puzzles)/sizeof(SUDOKU_PUZZLES); Index++) 
    {
        hSudoku = Sudoku_Init(g_Puzzles[Index].SudokuArray, FS_PRIORITY_NUM_COMPLETED_ROWS | FS_PRIORITY_INTERRUPTS);
        Sudoku_Solve(hSudoku);
        Sudoku_Debug_Console_Display(hSudoku);
        Sudoku_Free(hSudoku);

        hSudoku = Sudoku_Init(g_Puzzles[Index].SudokuArray, FS_PRIORITY_NUM_COMPLETED_COLS | FS_PRIORITY_INTERRUPTS);
        Sudoku_Solve(hSudoku);
        Sudoku_Debug_Console_Display(hSudoku);
        Sudoku_Free(hSudoku);

        hSudoku = Sudoku_Init(g_Puzzles[Index].SudokuArray, FS_PRIORITY_NUM_COMPLETED_NODES | FS_PRIORITY_INTERRUPTS);
        Sudoku_Solve(hSudoku);
        Sudoku_Debug_Console_Display(hSudoku);
        Sudoku_Free(hSudoku);
    }

    printf("Solving without Priority Interrupts\n");
    for(Index = 0; Index < sizeof(g_Puzzles)/sizeof(SUDOKU_PUZZLES); Index++) 
    {
        hSudoku = Sudoku_Init(g_Puzzles[Index].SudokuArray, 0);
        Sudoku_Debug_Console_Display(hSudoku);
        Sudoku_Solve(hSudoku);
        Sudoku_Debug_Console_Display(hSudoku);
        Sudoku_Free(hSudoku);
    }


    printf("Solving all possibilities\n");
    for(Index = 0; Index < sizeof(g_Puzzles)/sizeof(SUDOKU_PUZZLES); Index++) 
    {
        hSudoku = Sudoku_Init(g_Puzzles[Index].SudokuArray, SF_SOLVE_ALL_PATHS);
        Sudoku_Debug_Console_Display(hSudoku);
        Sudoku_Solve(hSudoku);
        Sudoku_Debug_Console_Display(hSudoku);
        Sudoku_Free(hSudoku);
    }

    return 0;
}

