/***************************************************************************** 
 *                            Sudoku Solver                                  *
 *                                                                           *
 *                                                                           *
 *                                                                           *
 *    Copyright (c)2013           Toby Opferman                              *
 *****************************************************************************/

#ifndef __SUDOKU_H__
#define __SUDOKU_H__

typedef PVOID HSUDOKU;

#define SF_SOLVE_ALL_PATHS                    0x1
#define FS_PRIORITY_INTERRUPTS                0x2
#define FS_PRIORITY_NUM_COMPLETED_ROWS        0x4
#define FS_PRIORITY_NUM_COMPLETED_COLS        0x8
#define FS_PRIORITY_NUM_COMPLETED_NODES       0x0
#define FS_PRIORITY_MASK 0xC

HSUDOKU Sudoku_Init(ULONG *pSudokuTable, ULONG Flags);
BOOL Sudoku_Solve(HSUDOKU hSudoku);
void Sudoku_Free(HSUDOKU hSudoku);
void Sudoku_Debug_Console_Display(HSUDOKU hSudoku);

#endif