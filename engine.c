/***************************************************************************** 
 *                            Sudoku Solver                                  *
 *                                                                           *
 *                                                                           *
 *                                                                           *
 *    Copyright (c)2013           Toby Opferman                              *
 *****************************************************************************/


#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <conio.h>
#include "sudoku.h"

/*
 * Function Macros
 */
#define Sudoku_SetNodeComplete(b, m) (b |= m)
#define Sudoku_SetNodesDirty(b, m) (b |= m)
#define Sudoku_IsNodeDirty(b, x, y) (b & (1<<((y*3)+x)))
#define Sudoku_ClearNodeDirty(b, x, y) (b &= ~(1<<((y*3)+x)))
#define Sudoku_IsOneBitSet(x) (((x-1) & (x))==0)

/*
 * Constant Definitions
 */
#define SUDOKU_COMPLETE_MASK  (0x1FF)
#define MAX_THREADS 8
#define NUM_WORKER_QUEUES 9
#define FLAG_SKIP_COMPLETE_NODES 0x1

#define Sudoku_Debug
#define Sudoku_Debug_Info
#define Sudoku_Debug_States 
#define Sudoku_Debug_Updates 

/*
 * State Machine.  Lower States can Pre-Empt higher states
 */
typedef enum _SUDOKU_STATE 
{
    SudokuStateBug = 0,
    SudokuStateFailure,
    SudokuStateSolved,
    SudokuStateProcessDirtyNodes,
    SudokuStateVerifyConsistency,
    SudokuStateFindEasyMatches,
    SudokuStateFindOnlyPossibleInNodeMatches,
    SudokuStateCondenseRows,
    SudokuStateCondenseCols,
    SudokuStateCondenseSuperRows,
    SudokuStateCondenseSuperCols,
    SudokuStateCondenseNodeCells,
    SudokuStateSuperCondenseNodeCells,
    SudokuStateSplit

} SUDOKU_STATE, *PSUDOKU_STATE;

/*
 * Default "TO-FROM" State Transition Table
 */
SUDOKU_STATE g_SudokuNextStateMapping[] =
{
    SudokuStateBug,
    SudokuStateFailure,
    SudokuStateSolved,
    SudokuStateVerifyConsistency,
    SudokuStateFindEasyMatches,
    SudokuStateFindOnlyPossibleInNodeMatches,
    SudokuStateCondenseRows,
    SudokuStateCondenseCols,
    SudokuStateCondenseSuperRows,
    SudokuStateCondenseSuperCols,
    SudokuStateCondenseNodeCells,
    SudokuStateSuperCondenseNodeCells,
    SudokuStateSplit,
    SudokuStateSplit
}; 

/*
 * Counter Tracking
 */
typedef enum _COUNTER_TYPE
{
   CounterTypeBug,
   CounterTypeFailure,
   CounterTypeSplit,
   CounterTypeDirections,
   CounterTypeStateSwitch

} COUNTER_TYPE, *PCOUNTER_TYPE;


/*
 * Internal Data Structures
 */
typedef struct _SUDOKU_COUNTERS
{
    ULONG Failures;
    ULONG Splits;
    ULONG Directions;
    ULONG Bugs;
    ULONG StateSwitches;

} SUDOKU_COUNTERS, *PSUDOKU_COUNTERS;

typedef struct _SUDOKU_THREAD_COUNTERS
{
    ULONG Interrupts;
    ULONG WorkItemsScheduled[NUM_WORKER_QUEUES];

} SUDOKU_THREAD_COUNTERS, *PSUDOKU_THREAD_COUNTERS;


typedef struct _TIME_TRACKING
{
   LARGE_INTEGER StartTime;
   LARGE_INTEGER StopTime;
   LARGE_INTEGER Frequency;
   float Seconds;
   float Microseconds;
   float Milliseconds;

} TIME_TRACKING, *PTIME_TRACKING;

typedef struct _SUDOKU_CELL
{
    BOOL Complete;

    ULONG PossibleMask;
    ULONG Number;

    ULONG NodesDirtyMask;

    ULONG *pCompletedRowLookup;
    ULONG *pCompletedColLookup;

} SUDOKU_CELL, *PSUDOKU_CELL;

typedef struct _SUDOKU_NODE
{
    BOOL Complete;
    SUDOKU_CELL Cell[3][3];

    ULONG CompletedMask;

    ULONG NodeBitmask;

} SUDOKU_NODE, *PSUDOKU_NODE;


struct _INTERNAL_SUDOKU;
typedef struct _INTERNAL_SUDOKU INTERNAL_SUDOKU;
typedef struct _INTERNAL_SUDOKU *PINTERNAL_SUDOKU;

typedef struct _PSUDOKU_GLOBAL_SYNC
{
    CRITICAL_SECTION csEngineLock;

    BOOL StopAllEngines;
    HANDLE hEngineComplete;
    HANDLE hWorkAvailable;
    HANDLE hThreadWorkerPool[MAX_THREADS];
    ULONG ConcurrentThreads;

    PINTERNAL_SUDOKU pSuccessConfiguration;

    PINTERNAL_SUDOKU pWorkerQueue[NUM_WORKER_QUEUES];

    SUDOKU_COUNTERS TotalEngineCounters;
    SUDOKU_THREAD_COUNTERS ThreadCounters;

} SUDOKU_GLOBAL_SYNC, *PSUDOKU_GLOBAL_SYNC;

typedef struct _INTERNAL_SUDOKU
{
    BOOL MasterEngine;
    ULONG Flags;
    BOOL StopEngine;
    BOOL SolveAttempted;
    BOOL Solved;
    SUDOKU_STATE State;
    SUDOKU_STATE NextState;
    SUDOKU_NODE Node[3][3];

    ULONG NumberOfCompletedNodes;  
    ULONG NumberOfCompletedRows;    
    ULONG NumberOfCompletedCols;    

    ULONG DirtyNodesBitmask;
    ULONG CompleteNodesBitmask;
    ULONG Priority;

    ULONG CompletedRowLookup[9];
    ULONG CompletedColLookup[9];

    PSUDOKU_GLOBAL_SYNC pSudokuGlobalSync;

    PINTERNAL_SUDOKU pWorkerQueueNext;
    PINTERNAL_SUDOKU pSolutionQueueNext;

    SUDOKU_COUNTERS EngineCounters;

    TIME_TRACKING TimeTracking;

} INTERNAL_SUDOKU, *PINTERNAL_SUDOKU;

typedef struct _NODE_CELL_LOCATION
{
    ULONG NodeX;
    ULONG NodeY;
    ULONG CellX;
    ULONG CellY;

} NODE_CELL_LOCATION, *PNODE_CELL_LOCATION;


typedef struct _NODE_CELL_ENUMERATION_CTX
{
    NODE_CELL_LOCATION NodeCellLocation;
    PINTERNAL_SUDOKU pSudoku;
    PSUDOKU_NODE pNode;
    PSUDOKU_CELL pCell;
    PVOID pContext;

} NODE_CELL_ENUMERATION_CTX, *PNODE_CELL_ENUMERATION_CTX;

typedef struct _SUDOKU_CONSISTENCY
{
    ULONG NodeCompleteMatrix[3][3];
    ULONG NodePossibleMatrix[3][3];

    ULONG RowComplete[9];
    ULONG RowPossible[9];

    ULONG ColComplete[9];
    ULONG ColPossible[9];

} SUDOKU_CONSISTENCY, *PSUDOKU_CONSISTENCY;

typedef struct _SUDOKU_FIND_BEST_CELL
{
    ULONG NumberOfPossibilities;

    ULONG NumberOfCompletedCellsInNode;
    ULONG NumberOfCompletedCellsInRow;
    ULONG NumberOfCompletedCellsInCol;

    ULONG TotalCompleted;
    ULONG MostCompleted;

    PSUDOKU_CELL pBestCell;

    NODE_CELL_LOCATION CellLocation;

} SUDOKU_FIND_BEST_CELL, *PSUDOKU_FIND_BEST_CELL;



typedef BOOL (*PFN_ENUMNODESANDCELLS)(PNODE_CELL_ENUMERATION_CTX pNodeCellEnumeration);

/*******************************************************************************
 * Internal Prototypes                                                         *
 *******************************************************************************/
void Sudoku_ProcessDirtyNode(PINTERNAL_SUDOKU pSudoku, ULONG NodeX, ULONG NodeY);
void Sudoku_UpdateCellPossibilityMasks(PINTERNAL_SUDOKU pSudoku, PSUDOKU_NODE pSudokuNode);
PINTERNAL_SUDOKU Sudoku_StateEngine(PINTERNAL_SUDOKU pSudoku);
BOOL Sudoku_StillRunning(PINTERNAL_SUDOKU *pSudoku);
void Sudoku_ProcessEasyMatches(PINTERNAL_SUDOKU pSudoku);
void Sudoku_ProcessEasyMatchesInNode(PINTERNAL_SUDOKU pSudoku, PSUDOKU_NODE pSudokuNode);
void Sudoku_CompleteCell(PINTERNAL_SUDOKU pSudoku, PSUDOKU_NODE pSudokuNode, PSUDOKU_CELL pSudokuCell);
void Sudoku_ProcessCondenseRows(PINTERNAL_SUDOKU pSudoku);
void Sudoku_ProcessCondenseCols(PINTERNAL_SUDOKU pSudoku);
void Sudoku_ProcessSuperCondenseRows(PINTERNAL_SUDOKU pSudoku);
void Sudoku_ProcessSuperCondenseCols(PINTERNAL_SUDOKU pSudoku);
void Sudoku_ProcessSplit(PINTERNAL_SUDOKU pSudoku);
void Sudoku_ProcessBug(PINTERNAL_SUDOKU pSudoku);
void Sudoku_ProcessSolved(PINTERNAL_SUDOKU pSudoku);
void Sudoku_InitializeNodesAndCells(PINTERNAL_SUDOKU pSudoku);
ULONG Sudoku_NodeCellIndexesToRowMask(ULONG NodeIndexX, ULONG NodeIndexY);
ULONG Sudoku_NodeCellIndexesToColMask(ULONG NodeIndexX, ULONG NodeIndexY);
void Sudoku_GetNodeCellLocation(ULONG IndexX, ULONG IndexY, PNODE_CELL_LOCATION pNodeCellLocation);
ULONG Sudoku_SetBitToNumber(ULONG Mask);
void Sudoku_Debug_DisplayMatrix(PINTERNAL_SUDOKU pSudoku);
BOOL Sudoku_Debug_Print(PNODE_CELL_ENUMERATION_CTX pNodeCellEnumeration);
void Sudoku_ProcessOnlyPossibleInNodeMatches(PINTERNAL_SUDOKU pSudoku);
void Sudoku_EnumNodesAndCellsByRows(PINTERNAL_SUDOKU pSudoku, PFN_ENUMNODESANDCELLS pEnumNodesAndCells, PVOID pContext);
void Sudoku_EnumNodesAndCellsByNodes(PINTERNAL_SUDOKU pSudoku, PFN_ENUMNODESANDCELLS pEnumNodesAndCells, PVOID pContext, ULONG Flags);
void Sudoku_EnumCellsByNode(PINTERNAL_SUDOKU pSudoku, PSUDOKU_NODE pNode, PFN_ENUMNODESANDCELLS pEnumNodesAndCells, PVOID pContext);
BOOL Sudoku_ProcessOnlyPossibleInNodeMatches_Callback(PNODE_CELL_ENUMERATION_CTX pNodeCellEnumeration);
BOOL Sudoku_ProcessOnlyPossibleInNodeMatches_CompareCells_Callback(PNODE_CELL_ENUMERATION_CTX pNodeCellEnumeration);
BOOL Sudoku_ProcessCondenseRows_Callback(PNODE_CELL_ENUMERATION_CTX pNodeCellEnumeration);
BOOL Sudoku_ProcessCondenseCols_Callback(PNODE_CELL_ENUMERATION_CTX pNodeCellEnumeration);
BOOL Sudoku_ProcessCondenseRows_CompareCells_Callback(PNODE_CELL_ENUMERATION_CTX pNodeCellEnumeration);
BOOL Sudoku_ProcessCondenseCols_CompareCells_Callback(PNODE_CELL_ENUMERATION_CTX pNodeCellEnumeration);
BOOL Sudoku_ProcessSuperCondenseRows_CompareCells_Callback(PNODE_CELL_ENUMERATION_CTX pNodeCellEnumeration);
BOOL Sudoku_ProcessSuperCondenseCols_CompareCells_Callback(PNODE_CELL_ENUMERATION_CTX pNodeCellEnumeration);
void Sudoku_ProcessVerifyConsistency(PINTERNAL_SUDOKU pSudoku);
void Sudoku_ProcessFailure(PINTERNAL_SUDOKU pSudoku);
void Sudoku_EnumNodesAndCellsByCols(PINTERNAL_SUDOKU pSudoku, PFN_ENUMNODESANDCELLS pEnumNodesAndCells, PVOID pContext);
BOOL Sudoku_Debug_PrintEx(PNODE_CELL_ENUMERATION_CTX pNodeCellEnumeration);
BOOL Sudoku_FindBestSplitCell_Callback(PNODE_CELL_ENUMERATION_CTX pNodeCellEnumeration);
BOOL Sudoku_Consistency_Callback(PNODE_CELL_ENUMERATION_CTX pNodeCellEnumeration);
ULONG Sudoku_NumerOfBits(ULONG BitMask);
void Sudoku_ProcessCondenseNodeCells(PINTERNAL_SUDOKU pSudoku);
void Sudoku_ProcessSuperCondenseNodeCells(PINTERNAL_SUDOKU pSudoku);
BOOL Sudoku_ProcessCondenseNodeCells_Callback(PNODE_CELL_ENUMERATION_CTX pNodeCellEnumeration);
BOOL Sudoku_ProcessSuperCondenseNodeCells_Callback(PNODE_CELL_ENUMERATION_CTX pNodeCellEnumeration);
DWORD Sudoku_SplitEngineWorkerThread(PVOID pEngineContext);
void Sudoku_MasterThreadEngine(PINTERNAL_SUDOKU pSudoku);
PSUDOKU_GLOBAL_SYNC Sudoku_CreateGlobalSyncContext(void);
PSUDOKU_CELL Sudoku_FindBestSplitCell(PINTERNAL_SUDOKU pSudoku, PNODE_CELL_LOCATION pCellLocation);
void Sudoku_AddWorkToQueue(PINTERNAL_SUDOKU pSudoku, PSUDOKU_CELL pBestCell, PNODE_CELL_LOCATION pCellLocation);
PINTERNAL_SUDOKU Sudoku_CreateDuplicateContext(PINTERNAL_SUDOKU pSudoku);
void Sudoku_NextState(PINTERNAL_SUDOKU pSudoku);
void Sudoku_SetNextState(PINTERNAL_SUDOKU pSudoku, SUDOKU_STATE SudokuState);
void Sudoku_MaskCellPossibilities(PINTERNAL_SUDOKU pSudoku, PSUDOKU_CELL pSudokuCell, ULONG Mask);
void Sudoku_StopEngine(PINTERNAL_SUDOKU pSudoku);
void Sudoku_StopAllEngines(PINTERNAL_SUDOKU pSudoku);
void Sudoku_UpdateCounter(PINTERNAL_SUDOKU pSudoku, COUNTER_TYPE CounterType, ULONG Count);
void Sudoku_PreserveTotalEngineCounters(PINTERNAL_SUDOKU pSudoku);
void Sudoku_Free(HSUDOKU hSudoku);
void Sudoku_DisplayCounters(PINTERNAL_SUDOKU pSudoku);
void Sudoku_StartTimer(PTIME_TRACKING pTimeTracking);
void Sudoku_StopTimer(PTIME_TRACKING pTimeTracking);
void Sudoku_DisplayTime(PTIME_TRACKING pTimeTracking);
BOOL Sudoku_WorkerQueuesAreEmpty(PSUDOKU_GLOBAL_SYNC pSudokuEngine, ULONG LowestPriority);
PINTERNAL_SUDOKU Sudoku_GetNextWorkItem(PSUDOKU_GLOBAL_SYNC pSudokuEngine, ULONG LowestPriority);
void Sudoku_PushWorkItem(PSUDOKU_GLOBAL_SYNC pSudokuEngine, PINTERNAL_SUDOKU pSudoku);
void Sudoku_UpdateContextPriority(PINTERNAL_SUDOKU pSudoku);


/******************************************************************************
 * Sudoku_Init                                                                        
 *                                                                             
 * DESCRIPTION: 
 *                                                                             
 * INPUT                                                                       
 *   
 *                                                                             
 * OUTPUT                                                                       
 *   
 *                                                                             
 *******************************************************************************/
HSUDOKU Sudoku_Init(ULONG *pSudokuTable, ULONG Flags)
{
    PINTERNAL_SUDOKU pInternalSudoku;
    ULONG IndexX;
    ULONG IndexY;
    NODE_CELL_LOCATION NodeCellLocation;

    pInternalSudoku = (PINTERNAL_SUDOKU)LocalAlloc(LMEM_ZEROINIT, sizeof(INTERNAL_SUDOKU));

    if(pInternalSudoku)
    {
        Sudoku_InitializeNodesAndCells(pInternalSudoku);

        pInternalSudoku->Flags = Flags;
        pInternalSudoku->MasterEngine = TRUE;

        for(IndexY = 0; IndexY < 9; IndexY++)
        {
            for(IndexX = 0; IndexX < 9; IndexX++)
            {
                if(pSudokuTable[IndexY*9 + IndexX] != 0)
                {
                    Sudoku_GetNodeCellLocation(IndexX,IndexY,&NodeCellLocation);
                    pInternalSudoku->Node[NodeCellLocation.NodeY][NodeCellLocation.NodeX].Cell[NodeCellLocation.CellY][NodeCellLocation.CellX].PossibleMask = (1<<(pSudokuTable[IndexY*9 + IndexX]-1));
                    Sudoku_CompleteCell(pInternalSudoku, &pInternalSudoku->Node[NodeCellLocation.NodeY][NodeCellLocation.NodeX], &pInternalSudoku->Node[NodeCellLocation.NodeY][NodeCellLocation.NodeX].Cell[NodeCellLocation.CellY][NodeCellLocation.CellX]);
                }
            }
        }

        pInternalSudoku->State = SudokuStateProcessDirtyNodes;
        pInternalSudoku->NextState = SudokuStateVerifyConsistency;
    }
    
    return (HSUDOKU)pInternalSudoku;
}


/*******************************************************************************
 * Sudoku_Solve                                                                        
 *                                                                             
 * DESCRIPTION: 
 *                                                                             
 * INPUT                                                                       
 *   
 *                                                                             
 * OUTPUT                                                                       
 *   
 *                                                                             
 *******************************************************************************/
BOOL Sudoku_Solve(HSUDOKU hSudoku)
{
    PINTERNAL_SUDOKU pInternalSudoku = (PINTERNAL_SUDOKU)hSudoku;

    if(pInternalSudoku->SolveAttempted == FALSE)
    {
        pInternalSudoku->SolveAttempted = TRUE;

        Sudoku_StartTimer(&pInternalSudoku->TimeTracking);
        Sudoku_StateEngine(pInternalSudoku);
        Sudoku_StopTimer(&pInternalSudoku->TimeTracking);
    }

    return pInternalSudoku->Solved;
}



/*******************************************************************************
 * Sudoku_Free                                                                        
 *                                                                             
 * DESCRIPTION: 
 *                                                                             
 * INPUT                                                                       
 *   
 *                                                                             
 * OUTPUT                                                                       
 *   
 *                                                                             
 *******************************************************************************/
void Sudoku_Free(HSUDOKU hSudoku)
{
    PINTERNAL_SUDOKU pInternalSudoku = (PINTERNAL_SUDOKU)hSudoku;
    PINTERNAL_SUDOKU pSudokuUncompletedWorkItem;
    UINT Index;

    if(pInternalSudoku->pSudokuGlobalSync)
    {
        if(pInternalSudoku->pSudokuGlobalSync->pSuccessConfiguration)
        {
            LocalFree(pInternalSudoku->pSudokuGlobalSync->pSuccessConfiguration);
        }

        for(Index = 0; Index < NUM_WORKER_QUEUES; Index++)
        {
            while(pInternalSudoku->pSudokuGlobalSync->pWorkerQueue[Index])
            {
                pSudokuUncompletedWorkItem = pInternalSudoku->pSudokuGlobalSync->pWorkerQueue[Index];
                pInternalSudoku->pSudokuGlobalSync->pWorkerQueue[Index] = pInternalSudoku->pSudokuGlobalSync->pWorkerQueue[Index]->pWorkerQueueNext;
                LocalFree(pSudokuUncompletedWorkItem);
            }
        }

        LocalFree(pInternalSudoku->pSudokuGlobalSync);
    }
    
    LocalFree(pInternalSudoku);
}


/*******************************************************************************
 * Sudoku_ProcessDirtyNodes                                                                        
 *                                                                             
 * DESCRIPTION: 
 *                                                                             
 * INPUT                                                                       
 *   
 *                                                                             
 * OUTPUT                                                                       
 *   
 *                                                                             
 *******************************************************************************/
void Sudoku_ProcessDirtyNodes(PINTERNAL_SUDOKU pSudoku)
{
    ULONG IndexX;
    ULONG IndexY;

    for(IndexY = 0; IndexY < 3 && pSudoku->DirtyNodesBitmask; IndexY++)
    {
        for(IndexX = 0; IndexX < 3 && pSudoku->DirtyNodesBitmask; IndexX++)
        {
            if(Sudoku_IsNodeDirty(pSudoku->DirtyNodesBitmask, IndexX, IndexY))
            {
                if(pSudoku->Node[IndexY][IndexX].Complete == FALSE)
                {
                    Sudoku_UpdateCellPossibilityMasks(pSudoku, &pSudoku->Node[IndexY][IndexX]);
                }

                Sudoku_ClearNodeDirty(pSudoku->DirtyNodesBitmask, IndexX, IndexY);
            }
        }
    }
}


/*******************************************************************************
 * Sudoku_ProcessEasyMatches                                                                        
 *                                                                             
 * DESCRIPTION: 
 *                                                                             
 * INPUT                                                                       
 *   
 *                                                                             
 * OUTPUT                                                                       
 *   
 *                                                                             
 *******************************************************************************/
void Sudoku_ProcessEasyMatches(PINTERNAL_SUDOKU pSudoku)
{
    ULONG IndexX;
    ULONG IndexY;

    for(IndexY = 0; IndexY < 3; IndexY++)
    {
        for(IndexX = 0; IndexX < 3; IndexX++)
        {
            if(pSudoku->Node[IndexY][IndexX].Complete == FALSE)
            {
                Sudoku_ProcessEasyMatchesInNode(pSudoku, &pSudoku->Node[IndexY][IndexX]);
            }
        }
    }
}


/*******************************************************************************
 * Sudoku_ProcessCondenseCols_CompareCells_Callback                                                                        
 *                                                                             
 * DESCRIPTION: 
 *                                                                             
 * INPUT                                                                       
 *   
 *                                                                             
 * OUTPUT                                                                       
 *   
 *                                                                             
 *******************************************************************************/
BOOL Sudoku_ProcessCondenseCols_CompareCells_Callback(PNODE_CELL_ENUMERATION_CTX pNodeCellEnumeration)
{
    PNODE_CELL_ENUMERATION_CTX pOriginalNodeEnum = (PNODE_CELL_ENUMERATION_CTX)pNodeCellEnumeration->pContext;
    ULONG PossibleMask;

    PossibleMask = *((ULONG *)pOriginalNodeEnum->pContext);

    if(pNodeCellEnumeration->pCell->Complete == FALSE)
    {
        if(pNodeCellEnumeration->NodeCellLocation.CellX != pOriginalNodeEnum->NodeCellLocation.CellX)
        {
            if((pNodeCellEnumeration->pCell->PossibleMask & PossibleMask) == PossibleMask)
            {
                Sudoku_Debug(" Removing Mask(0x%0x) from Mask (0x%0x) Node (%i, %i) Cell (%i, %i)", PossibleMask, pNodeCellEnumeration->pCell->PossibleMask, pNodeCellEnumeration->NodeCellLocation.NodeX, pNodeCellEnumeration->NodeCellLocation.NodeY, pNodeCellEnumeration->NodeCellLocation.CellX, pNodeCellEnumeration->NodeCellLocation.CellY);

                Sudoku_MaskCellPossibilities(pNodeCellEnumeration->pSudoku, pNodeCellEnumeration->pCell, (~PossibleMask));
            }

        }
    }

    return TRUE;
}


/*******************************************************************************
 * Sudoku_ProcessCondenseRows_CompareCells_Callback                                                                        
 *                                                                             
 * DESCRIPTION: 
 *                                                                             
 * INPUT                                                                       
 *   
 *                                                                             
 * OUTPUT                                                                       
 *   
 *                                                                             
 *******************************************************************************/
BOOL Sudoku_ProcessCondenseRows_CompareCells_Callback(PNODE_CELL_ENUMERATION_CTX pNodeCellEnumeration)
{
    PNODE_CELL_ENUMERATION_CTX pOriginalNodeEnum = (PNODE_CELL_ENUMERATION_CTX)pNodeCellEnumeration->pContext;
    ULONG PossibleMask;

    PossibleMask = *((ULONG *)pOriginalNodeEnum->pContext);

    if(pNodeCellEnumeration->pCell->Complete == FALSE)
    {
        if(pNodeCellEnumeration->NodeCellLocation.CellY != pOriginalNodeEnum->NodeCellLocation.CellY)
        {
            if((pNodeCellEnumeration->pCell->PossibleMask & PossibleMask) == PossibleMask)
            {
                Sudoku_Debug(" Removing Mask(0x%0x) from Mask (0x%0x) Node (%i, %i) Cell (%i, %i)", PossibleMask, pNodeCellEnumeration->pCell->PossibleMask, pNodeCellEnumeration->NodeCellLocation.NodeX, pNodeCellEnumeration->NodeCellLocation.NodeY, pNodeCellEnumeration->NodeCellLocation.CellX, pNodeCellEnumeration->NodeCellLocation.CellY);

                Sudoku_MaskCellPossibilities(pNodeCellEnumeration->pSudoku, pNodeCellEnumeration->pCell, (~PossibleMask));
            }

        }
    }

    return TRUE;
}

/*******************************************************************************
 * Sudoku_ProcessOnlyPossibleInNodeMatches_CompareCells_Callback                                                                        
 *                                                                             
 * DESCRIPTION: 
 *                                                                             
 * INPUT                                                                       
 *   
 *                                                                             
 * OUTPUT                                                                       
 *   
 *                                                                             
 *******************************************************************************/
BOOL Sudoku_ProcessOnlyPossibleInNodeMatches_CompareCells_Callback(PNODE_CELL_ENUMERATION_CTX pNodeCellEnumeration)
{
    PNODE_CELL_ENUMERATION_CTX pOriginalNodeEnum = (PNODE_CELL_ENUMERATION_CTX)pNodeCellEnumeration->pContext;
    ULONG *PossibleMask = (ULONG *)pOriginalNodeEnum->pContext;
    BOOL ContinueProcessing;

    ContinueProcessing = TRUE;

    if(pNodeCellEnumeration->pCell->Complete == FALSE)
    {
        if(pNodeCellEnumeration->pCell != pOriginalNodeEnum->pCell)
        {
            Sudoku_Debug("       Node(%i, %i) Cell(%i, %i) Mask(0x%0x)\n", pNodeCellEnumeration->NodeCellLocation.NodeX, pNodeCellEnumeration->NodeCellLocation.NodeY, pNodeCellEnumeration->NodeCellLocation.CellX, pNodeCellEnumeration->NodeCellLocation.CellY, pNodeCellEnumeration->pCell->PossibleMask);
            *PossibleMask = (*PossibleMask ^ pNodeCellEnumeration->pCell->PossibleMask) & *PossibleMask;

            if(*PossibleMask == 0)
            {
                ContinueProcessing = FALSE;
            }
        }
    }

    return ContinueProcessing;
}

/*******************************************************************************
 * Sudoku_ProcessOnlyPossibleInNodeMatches_Callback                                                                        
 *                                                                             
 * DESCRIPTION: 
 *                                                                             
 * INPUT                                                                       
 *   
 *                                                                             
 * OUTPUT                                                                       
 *   
 *                                                                             
 *******************************************************************************/
BOOL Sudoku_ProcessOnlyPossibleInNodeMatches_Callback(PNODE_CELL_ENUMERATION_CTX pNodeCellEnumeration)
{
    ULONG PossibleMask;

    if(pNodeCellEnumeration->pCell->Complete == FALSE)
    {
        PossibleMask = pNodeCellEnumeration->pCell->PossibleMask;

        Sudoku_Debug("Node(%i, %i) Cell(%i, %i) Mask(0x%0x)\n", pNodeCellEnumeration->NodeCellLocation.NodeX, pNodeCellEnumeration->NodeCellLocation.NodeY, pNodeCellEnumeration->NodeCellLocation.CellX, pNodeCellEnumeration->NodeCellLocation.CellY, pNodeCellEnumeration->pCell->PossibleMask);

        pNodeCellEnumeration->pContext = &PossibleMask;

        Sudoku_EnumCellsByNode(pNodeCellEnumeration->pSudoku, pNodeCellEnumeration->pNode, Sudoku_ProcessOnlyPossibleInNodeMatches_CompareCells_Callback, pNodeCellEnumeration);

        if(PossibleMask)
        {
            Sudoku_Debug("New Possible Mask 0x%0x\n\n", PossibleMask);

            pNodeCellEnumeration->pCell->PossibleMask = PossibleMask;
            Sudoku_CompleteCell(pNodeCellEnumeration->pSudoku, pNodeCellEnumeration->pNode, pNodeCellEnumeration->pCell);
        }
    }

    return TRUE;
}


/*******************************************************************************
 * Sudoku_ProcessCondenseRows_Callback                                                                        
 *                                                                             
 * DESCRIPTION: 
 *                                                                             
 * INPUT                                                                       
 *   
 *                                                                             
 * OUTPUT                                                                       
 *   
 *                                                                             
 *******************************************************************************/
BOOL Sudoku_ProcessCondenseRows_Callback(PNODE_CELL_ENUMERATION_CTX pNodeCellEnumeration)
{
    ULONG PossibleMask;
    ULONG RowIndex;
    ULONG NodeIndex;

    if(pNodeCellEnumeration->pCell->Complete == FALSE)
    {
        PossibleMask = pNodeCellEnumeration->pCell->PossibleMask;

        Sudoku_Debug("Node(%i, %i) Cell(%i, %i) Mask(0x%0x)\n", pNodeCellEnumeration->NodeCellLocation.NodeX, pNodeCellEnumeration->NodeCellLocation.NodeY, pNodeCellEnumeration->NodeCellLocation.CellX, pNodeCellEnumeration->NodeCellLocation.CellY, pNodeCellEnumeration->pCell->PossibleMask);

        for(NodeIndex = 0; NodeIndex < 3 && PossibleMask; NodeIndex++)
        {
            if(pNodeCellEnumeration->NodeCellLocation.NodeX != NodeIndex)
            {
                if(pNodeCellEnumeration->pSudoku->Node[pNodeCellEnumeration->NodeCellLocation.NodeY][NodeIndex].Complete == FALSE)
                {
                    for(RowIndex = 0; RowIndex < 3 && PossibleMask; RowIndex++)
                    {
                        if(pNodeCellEnumeration->pSudoku->Node[pNodeCellEnumeration->NodeCellLocation.NodeY][NodeIndex].Cell[pNodeCellEnumeration->NodeCellLocation.CellY][RowIndex].Complete == FALSE)
                        {
                            Sudoku_Debug("       Node(%i, %i) Cell(%i, %i) Mask(0x%0x)\n", NodeIndex, pNodeCellEnumeration->NodeCellLocation.NodeY, RowIndex, pNodeCellEnumeration->NodeCellLocation.CellY, pNodeCellEnumeration->pSudoku->Node[pNodeCellEnumeration->NodeCellLocation.NodeY][NodeIndex].Cell[pNodeCellEnumeration->NodeCellLocation.CellY][RowIndex].PossibleMask);

                            PossibleMask = (PossibleMask ^ pNodeCellEnumeration->pSudoku->Node[pNodeCellEnumeration->NodeCellLocation.NodeY][NodeIndex].Cell[pNodeCellEnumeration->NodeCellLocation.CellY][RowIndex].PossibleMask) & PossibleMask;
                        }
                    }
                }
            }
        }

        if(PossibleMask)
        {
            Sudoku_Debug("New Possible Mask 0x%0x\n\n", PossibleMask);
            pNodeCellEnumeration->pContext = &PossibleMask;
            Sudoku_EnumCellsByNode(pNodeCellEnumeration->pSudoku, pNodeCellEnumeration->pNode, Sudoku_ProcessCondenseRows_CompareCells_Callback, pNodeCellEnumeration);
        }
    }

    return TRUE;
}




/*******************************************************************************
 * Sudoku_ProcessCondenseCols_Callback                                                                        
 *                                                                             
 * DESCRIPTION: 
 *                                                                             
 * INPUT                                                                       
 *   
 *                                                                             
 * OUTPUT                                                                       
 *   
 *                                                                             
 *******************************************************************************/
BOOL Sudoku_ProcessCondenseCols_Callback(PNODE_CELL_ENUMERATION_CTX pNodeCellEnumeration)
{
    ULONG PossibleMask;
    ULONG ColIndex;
    ULONG NodeIndex;

    if(pNodeCellEnumeration->pCell->Complete == FALSE)
    {
        PossibleMask = pNodeCellEnumeration->pCell->PossibleMask;

        Sudoku_Debug("Node(%i, %i) Cell(%i, %i) Mask(0x%0x)\n", pNodeCellEnumeration->NodeCellLocation.NodeX, pNodeCellEnumeration->NodeCellLocation.NodeY, pNodeCellEnumeration->NodeCellLocation.CellX, pNodeCellEnumeration->NodeCellLocation.CellY, pNodeCellEnumeration->pCell->PossibleMask);

        for(NodeIndex = 0; NodeIndex < 3 && PossibleMask; NodeIndex++)
        {
            if(pNodeCellEnumeration->NodeCellLocation.NodeY != NodeIndex)
            {
                if(pNodeCellEnumeration->pSudoku->Node[NodeIndex][pNodeCellEnumeration->NodeCellLocation.NodeX].Complete == FALSE)
                {
                    for(ColIndex = 0; ColIndex < 3 && PossibleMask; ColIndex++)
                    {
                        if(pNodeCellEnumeration->pSudoku->Node[NodeIndex][pNodeCellEnumeration->NodeCellLocation.NodeX].Cell[ColIndex][pNodeCellEnumeration->NodeCellLocation.CellX].Complete == FALSE)
                        {
                            Sudoku_Debug("       Node(%i, %i) Cell(%i, %i) Mask(0x%0x)\n", pNodeCellEnumeration->NodeCellLocation.NodeX, NodeIndex, pNodeCellEnumeration->NodeCellLocation.CellX, ColIndex, pNodeCellEnumeration->pSudoku->Node[NodeIndex][pNodeCellEnumeration->NodeCellLocation.NodeX].Cell[ColIndex][pNodeCellEnumeration->NodeCellLocation.CellX].PossibleMask);

                            PossibleMask = (PossibleMask ^ pNodeCellEnumeration->pSudoku->Node[NodeIndex][pNodeCellEnumeration->NodeCellLocation.NodeX].Cell[ColIndex][pNodeCellEnumeration->NodeCellLocation.CellX].PossibleMask) & PossibleMask;
                        }
                    }
                }
            }
        }

        if(PossibleMask)
        {
            Sudoku_Debug("New Possible Mask 0x%0x\n\n", PossibleMask);
            pNodeCellEnumeration->pContext = &PossibleMask;
            Sudoku_EnumCellsByNode(pNodeCellEnumeration->pSudoku, pNodeCellEnumeration->pNode, Sudoku_ProcessCondenseCols_CompareCells_Callback, pNodeCellEnumeration);
        }
    }

    return TRUE;
}

/*******************************************************************************
 * Sudoku_ProcessOnlyPossibleInNodeMatches                                                                        
 *                                                                             
 * DESCRIPTION: 
 *                                                                             
 * INPUT                                                                       
 *   
 *                                                                             
 * OUTPUT                                                                       
 *   
 *                                                                             
 *******************************************************************************/
void Sudoku_ProcessOnlyPossibleInNodeMatches(PINTERNAL_SUDOKU pSudoku)
{
    Sudoku_EnumNodesAndCellsByNodes(pSudoku, Sudoku_ProcessOnlyPossibleInNodeMatches_Callback, NULL, FLAG_SKIP_COMPLETE_NODES);
}

/*******************************************************************************
 * Sudoku_UpdateCellPossibilityMasks                                                                        
 *                                                                             
 * DESCRIPTION: 
 *                                                                             
 * INPUT                                                                       
 *   
 *                                                                             
 * OUTPUT                                                                       
 *   
 *                                                                             
 *******************************************************************************/
void Sudoku_UpdateCellPossibilityMasks(PINTERNAL_SUDOKU pSudoku, PSUDOKU_NODE pSudokuNode)
{
    ULONG IndexX;
    ULONG IndexY;

    for(IndexX = 0; IndexX < 3; IndexX++)
    {
        for(IndexY = 0; IndexY < 3; IndexY++)
        {
            if(pSudokuNode->Cell[IndexY][IndexX].Complete == FALSE)
            {
                Sudoku_MaskCellPossibilities(pSudoku, &pSudokuNode->Cell[IndexY][IndexX], (~pSudokuNode->CompletedMask));
                Sudoku_MaskCellPossibilities(pSudoku, &pSudokuNode->Cell[IndexY][IndexX], (~(*pSudokuNode->Cell[IndexY][IndexX].pCompletedRowLookup)));
                Sudoku_MaskCellPossibilities(pSudoku, &pSudokuNode->Cell[IndexY][IndexX], (~(*pSudokuNode->Cell[IndexY][IndexX].pCompletedColLookup)));
            }
        }
    }
}


/*******************************************************************************
 * Sudoku_StillRunning                                                                        
 *                                                                             
 * DESCRIPTION: 
 *                                                                             
 * INPUT                                                                       
 *   
 *                                                                             
 * OUTPUT                                                                       
 *   
 *                                                                             
 *******************************************************************************/
BOOL Sudoku_StillRunning(PINTERNAL_SUDOKU *pSudoku)
{
   BOOL EngineRunning = TRUE;
   PINTERNAL_SUDOKU pSudokuNew;
   PINTERNAL_SUDOKU pSudokuCurrent;

   pSudokuCurrent = *pSudoku;

   if(pSudokuCurrent->MasterEngine == FALSE)
   {
       if(pSudokuCurrent->pSudokuGlobalSync->StopAllEngines)
       {
           EngineRunning = FALSE;
       }

       if((pSudokuCurrent->Flags & FS_PRIORITY_INTERRUPTS) != 0)
       {
           EnterCriticalSection(&pSudokuCurrent->pSudokuGlobalSync->csEngineLock);

           if(pSudokuCurrent->pSudokuGlobalSync->ConcurrentThreads == MAX_THREADS)
           {
               if(Sudoku_WorkerQueuesAreEmpty(pSudokuCurrent->pSudokuGlobalSync, pSudokuCurrent->Priority+1) == FALSE)
               {
                   pSudokuNew = Sudoku_GetNextWorkItem(pSudokuCurrent->pSudokuGlobalSync, pSudokuCurrent->Priority+1);
                   Sudoku_PushWorkItem(pSudokuCurrent->pSudokuGlobalSync, *pSudoku);
                   *pSudoku = pSudokuNew;
                   pSudokuCurrent = pSudokuNew;
                   pSudokuCurrent->pSudokuGlobalSync->ThreadCounters.Interrupts++;
               }
           }

           LeaveCriticalSection(&pSudokuCurrent->pSudokuGlobalSync->csEngineLock);
       }
   }

   if(pSudokuCurrent->StopEngine)
   {
       EngineRunning = FALSE;
   }

   return EngineRunning;
}


/*******************************************************************************
 * Sudoku_CompleteCell                                                                        
 *                                                                             
 * DESCRIPTION: 
 *                                                                             
 * INPUT                                                                       
 *   
 *                                                                             
 * OUTPUT                                                                       
 *   
 *                                                                             
 *******************************************************************************/
void Sudoku_CompleteCell(PINTERNAL_SUDOKU pSudoku, PSUDOKU_NODE pSudokuNode, PSUDOKU_CELL pSudokuCell)
{

    if(pSudokuCell->PossibleMask != 0)
    {
        pSudokuCell->Complete = TRUE;
    
        *pSudokuCell->pCompletedColLookup |= pSudokuCell->PossibleMask;
        *pSudokuCell->pCompletedRowLookup |= pSudokuCell->PossibleMask;

        if((*pSudokuCell->pCompletedColLookup & SUDOKU_COMPLETE_MASK) == SUDOKU_COMPLETE_MASK)
        {
            pSudoku->NumberOfCompletedCols++;
        }

        if((*pSudokuCell->pCompletedRowLookup & SUDOKU_COMPLETE_MASK) == SUDOKU_COMPLETE_MASK)
        {
            pSudoku->NumberOfCompletedRows++;
        }

        pSudokuCell->Number = Sudoku_SetBitToNumber(pSudokuCell->PossibleMask);
    
        Sudoku_SetNextState(pSudoku, SudokuStateProcessDirtyNodes);
    
        pSudokuNode->CompletedMask |= pSudokuCell->PossibleMask;
    
        if((pSudokuNode->CompletedMask & SUDOKU_COMPLETE_MASK) == SUDOKU_COMPLETE_MASK)
        {
            Sudoku_Debug(" Setting Node Complete\n");
            pSudokuNode->Complete          = TRUE;
            pSudoku->CompleteNodesBitmask |= pSudokuNode->NodeBitmask;

            pSudoku->NumberOfCompletedNodes++;
    
            if((pSudoku->CompleteNodesBitmask & SUDOKU_COMPLETE_MASK) == SUDOKU_COMPLETE_MASK)
            {
                Sudoku_SetNextState(pSudoku, SudokuStateSolved);
            }
        }

        Sudoku_UpdateContextPriority(pSudoku);
    
        Sudoku_SetNodesDirty(pSudoku->DirtyNodesBitmask, pSudokuCell->NodesDirtyMask);
    }
    else
    {
        Sudoku_SetNextState(pSudoku, SudokuStateBug);
    }
}


/*******************************************************************************
 * Sudoku_ProcessEasyMatchesInNode                                                                        
 *                                                                             
 * DESCRIPTION: 
 *                                                                             
 * INPUT                                                                       
 *   
 *                                                                             
 * OUTPUT                                                                       
 *   
 *                                                                             
 *******************************************************************************/
void Sudoku_ProcessEasyMatchesInNode(PINTERNAL_SUDOKU pSudoku, PSUDOKU_NODE pSudokuNode)
{
    ULONG IndexX;
    ULONG IndexY;

    for(IndexX = 0; IndexX < 3; IndexX++)
    {
        for(IndexY = 0; IndexY < 3; IndexY++)
        {
            if(pSudokuNode->Cell[IndexY][IndexX].Complete == FALSE)
            {
                if(Sudoku_IsOneBitSet(pSudokuNode->Cell[IndexY][IndexX].PossibleMask))
                {
                    Sudoku_CompleteCell(pSudoku, pSudokuNode, &pSudokuNode->Cell[IndexY][IndexX]);
                }
            }
        }
    }
}


/*******************************************************************************
 * Sudoku_StateEngine                                                                        
 *                                                                             
 * DESCRIPTION: 
 *                                                                             
 * INPUT                                                                       
 *   
 *                                                                             
 * OUTPUT                                                                       
 *   
 *                                                                             
 *******************************************************************************/
PINTERNAL_SUDOKU Sudoku_StateEngine(PINTERNAL_SUDOKU pSudoku)
{

    while(Sudoku_StillRunning(&pSudoku))
    {
        switch(pSudoku->State)
        {
            case SudokuStateProcessDirtyNodes:
                 Sudoku_Debug_States(" State: SudokuStateProcessDirtyNodes\n");
                 Sudoku_ProcessDirtyNodes(pSudoku);
                 break;

            case SudokuStateVerifyConsistency:
                Sudoku_Debug_States(" State: SudokuStateVerifyConsistency\n");
                Sudoku_ProcessVerifyConsistency(pSudoku);
                break;

            case SudokuStateFindEasyMatches:
                 Sudoku_Debug_States(" State: SudokuStateFindEasyMatches\n");
                 Sudoku_ProcessEasyMatches(pSudoku);
                 break;

            case SudokuStateFindOnlyPossibleInNodeMatches:
                 Sudoku_Debug_States(" State: SudokuStateFindOnlyPossibleInNodeMatches\n");
                 Sudoku_ProcessOnlyPossibleInNodeMatches(pSudoku);
                 break;
                                                     
            case SudokuStateCondenseRows:
                 Sudoku_Debug_States(" State: SudokuStateCondenseRows\n");
                 Sudoku_ProcessCondenseRows(pSudoku);
                 break;

            case SudokuStateCondenseCols:
                 Sudoku_Debug_States(" State: SudokuStateCondenseCols\n");
                 Sudoku_ProcessCondenseCols(pSudoku);
                 break;

            case SudokuStateCondenseSuperRows:
                 Sudoku_Debug_States(" State: SudokuStateCondenseSuperRows\n");
                 Sudoku_ProcessSuperCondenseRows(pSudoku);
                 break;

            case SudokuStateCondenseSuperCols:
                 Sudoku_Debug_States(" State: SudokuStateCondenseSuperCols\n");
                 Sudoku_ProcessSuperCondenseCols(pSudoku);
                 break;

            case SudokuStateSplit:
                 Sudoku_Debug_States(" State: SudokuStateSplit\n");
                 Sudoku_ProcessSplit(pSudoku);
                 break;

            case SudokuStateFailure:
                 Sudoku_Debug_States(" State: SudokuStateFailure\n");
                 Sudoku_ProcessFailure(pSudoku);
                 break;

            case SudokuStateCondenseNodeCells:
                 Sudoku_Debug_States(" State: SudokuStateCondenseNodeCells\n");
                 Sudoku_ProcessCondenseNodeCells(pSudoku);
                 break;

            case SudokuStateSuperCondenseNodeCells:
                 Sudoku_Debug_States(" State: SudokuStateSuperCondenseNodeCells\n");
                 Sudoku_ProcessSuperCondenseNodeCells(pSudoku);
                 break;

            case SudokuStateSolved:
                 Sudoku_Debug_States(" State: SudokuStateSolved\n");
                 Sudoku_ProcessSolved(pSudoku);
                 break;

            case SudokuStateBug:
                Sudoku_Debug_States(" State: SudokuStateBug\n");
                Sudoku_ProcessBug(pSudoku);
                break;

            default:
                 Sudoku_SetNextState(pSudoku, SudokuStateBug);
        }

        Sudoku_NextState(pSudoku);
    }

    return pSudoku;
}


/*******************************************************************************
 * Sudoku_ProcessCondenseRows                                                                        
 *                                                                             
 * DESCRIPTION: 
 *                                                                             
 * INPUT                                                                       
 *   
 *                                                                             
 * OUTPUT                                                                       
 *   
 *                                                                             
 *******************************************************************************/
void Sudoku_ProcessCondenseRows(PINTERNAL_SUDOKU pSudoku)
{
    Sudoku_EnumNodesAndCellsByNodes(pSudoku, Sudoku_ProcessCondenseRows_Callback, NULL, FLAG_SKIP_COMPLETE_NODES);
}


/*******************************************************************************
 * Sudoku_ProcessCondenseCols                                                                        
 *                                                                             
 * DESCRIPTION: 
 *                                                                             
 * INPUT                                                                       
 *   
 *                                                                             
 * OUTPUT                                                                       
 *   
 *                                                                             
 *******************************************************************************/
void Sudoku_ProcessCondenseCols(PINTERNAL_SUDOKU pSudoku)
{
    Sudoku_EnumNodesAndCellsByNodes(pSudoku, Sudoku_ProcessCondenseCols_Callback, NULL, FLAG_SKIP_COMPLETE_NODES);
}


/*******************************************************************************
 * Sudoku_ProcessSplit                                                                        
 *                                                                             
 * DESCRIPTION: 
 *                                                                             
 * INPUT                                                                       
 *   
 *                                                                             
 * OUTPUT                                                                       
 *   
 *                                                                             
 *******************************************************************************/
void Sudoku_ProcessSplit(PINTERNAL_SUDOKU pSudoku)
{
    PSUDOKU_CELL pBestCell;
    NODE_CELL_LOCATION CellLocation;

    Sudoku_UpdateCounter(pSudoku, CounterTypeSplit, 1);

    if(pSudoku->MasterEngine)
    {
        Sudoku_MasterThreadEngine(pSudoku);
    }
    else
    {
        pBestCell = Sudoku_FindBestSplitCell(pSudoku, &CellLocation);
        Sudoku_AddWorkToQueue(pSudoku, pBestCell, &CellLocation);
    }
}


/*******************************************************************************
 * Sudoku_MasterThreadEngine                                                                        
 *                                                                             
 * DESCRIPTION: 
 *                                                                             
 * INPUT                                                                       
 *   
 *                                                                             
 * OUTPUT                                                                       
 *   
 *                                                                             
 *******************************************************************************/
void Sudoku_MasterThreadEngine(PINTERNAL_SUDOKU pSudoku)
{
    UINT ThreadIndex;
    PSUDOKU_CELL pBestCell;
    NODE_CELL_LOCATION CellLocation;

    pSudoku->pSudokuGlobalSync = Sudoku_CreateGlobalSyncContext();

    if(pSudoku->pSudokuGlobalSync)
    {
        for(ThreadIndex = 0; ThreadIndex < MAX_THREADS; ThreadIndex++)
        {
            pSudoku->pSudokuGlobalSync->hThreadWorkerPool[ThreadIndex] = CreateThread(NULL, 0, Sudoku_SplitEngineWorkerThread, pSudoku->pSudokuGlobalSync, 0, NULL);
        }
        
        pBestCell = Sudoku_FindBestSplitCell(pSudoku, &CellLocation);

        Sudoku_AddWorkToQueue(pSudoku, pBestCell, &CellLocation);

        WaitForSingleObject(pSudoku->pSudokuGlobalSync->hEngineComplete, INFINITE);

        WaitForMultipleObjects(MAX_THREADS, &pSudoku->pSudokuGlobalSync->hThreadWorkerPool[0], TRUE, INFINITE);
        
        if(pSudoku->pSudokuGlobalSync->pSuccessConfiguration)
        {
            pSudoku->Solved = TRUE;
        }

        Sudoku_StopEngine(pSudoku);
    }
}

       
/*******************************************************************************
 * Sudoku_AddWorkToQueue                                                                        
 *                                                                             
 * DESCRIPTION: 
 *                                                                             
 * INPUT                                                                       
 *   
 *                                                                             
 * OUTPUT                                                                       
 *   
 *                                                                             
 *******************************************************************************/
void Sudoku_AddWorkToQueue(PINTERNAL_SUDOKU pSudoku, PSUDOKU_CELL pBestCell, PNODE_CELL_LOCATION pCellLocation)
{
    ULONG NumberOfContexts;
    ULONG BitmaskSelection;
    PINTERNAL_SUDOKU pSudokuNewContext;
    ULONG SetBit;

    NumberOfContexts = Sudoku_NumerOfBits(pBestCell->PossibleMask);

    BitmaskSelection = pBestCell->PossibleMask;

    Sudoku_UpdateCounter(pSudoku, CounterTypeDirections, NumberOfContexts);

    if(pSudoku->MasterEngine == FALSE)
    {
        NumberOfContexts = NumberOfContexts - 1;
    }

    SetBit = 1;

    for(;NumberOfContexts > 0; NumberOfContexts--)
    {
         while((BitmaskSelection & 1) == 0)
         {
             SetBit <<= 1;
             BitmaskSelection >>=1;
         }

         EnterCriticalSection(&pSudoku->pSudokuGlobalSync->csEngineLock);
         pSudokuNewContext = Sudoku_CreateDuplicateContext(pSudoku);

         if(pSudokuNewContext)
         {
             pSudokuNewContext->Node[pCellLocation->NodeY][pCellLocation->NodeX].Cell[pCellLocation->CellY][pCellLocation->CellX].PossibleMask = SetBit;
             Sudoku_CompleteCell(pSudokuNewContext, &pSudokuNewContext->Node[pCellLocation->NodeY][pCellLocation->NodeX], &pSudokuNewContext->Node[pCellLocation->NodeY][pCellLocation->NodeX].Cell[pCellLocation->CellY][pCellLocation->CellX]);
             Sudoku_PushWorkItem(pSudoku->pSudokuGlobalSync, pSudokuNewContext);
         }

         LeaveCriticalSection(&pSudoku->pSudokuGlobalSync->csEngineLock);

         SetBit <<= 1;
         BitmaskSelection >>=1;
    }
        
    if(pSudoku->MasterEngine == FALSE)
    {
         while((BitmaskSelection & 1) == 0)
         {
             SetBit <<= 1;
             BitmaskSelection >>=1;
         }

         pSudoku->Node[pCellLocation->NodeY][pCellLocation->NodeX].Cell[pCellLocation->CellY][pCellLocation->CellX].PossibleMask = SetBit;
         Sudoku_CompleteCell(pSudoku, &pSudoku->Node[pCellLocation->NodeY][pCellLocation->NodeX], &pSudoku->Node[pCellLocation->NodeY][pCellLocation->NodeX].Cell[pCellLocation->CellY][pCellLocation->CellX]);         
    }
}


/*******************************************************************************
 * Sudoku_ProcessSolved                                                                        
 *                                                                             
 * DESCRIPTION: 
 *                                                                             
 * INPUT                                                                       
 *   
 *                                                                             
 * OUTPUT                                                                       
 *   
 *                                                                             
 *******************************************************************************/
void Sudoku_ProcessSolved(PINTERNAL_SUDOKU pSudoku)
{
    pSudoku->Solved = TRUE;
    Sudoku_StopAllEngines(pSudoku);
}


/*******************************************************************************
 * Sudoku_InitializeNodesAndCells                                                                        
 *                                                                             
 * DESCRIPTION: 
 *                                                                             
 * INPUT                                                                       
 *   
 *                                                                             
 * OUTPUT                                                                       
 *   
 *                                                                             
 *******************************************************************************/
void Sudoku_InitializeNodesAndCells(PINTERNAL_SUDOKU pSudoku)
{
    ULONG IndexX;
    ULONG IndexY;
    ULONG CellIndexX;
    ULONG CellIndexY;

    for(IndexY = 0; IndexY < 3; IndexY++)
    {
        for(IndexX = 0; IndexX < 3; IndexX++)
        {
            pSudoku->Node[IndexY][IndexX].NodeBitmask = 1<<(IndexX + (IndexY*3));

            for(CellIndexY = 0; CellIndexY < 3; CellIndexY++)
            {
                for(CellIndexX = 0; CellIndexX < 3; CellIndexX++)
                {
                    pSudoku->Node[IndexY][IndexX].Cell[CellIndexY][CellIndexX].PossibleMask = SUDOKU_COMPLETE_MASK;
                    pSudoku->Node[IndexY][IndexX].Cell[CellIndexY][CellIndexX].NodesDirtyMask = Sudoku_NodeCellIndexesToRowMask(IndexX, IndexY) | Sudoku_NodeCellIndexesToColMask(IndexX, IndexY);
                    pSudoku->Node[IndexY][IndexX].Cell[CellIndexY][CellIndexX].pCompletedColLookup = &pSudoku->CompletedColLookup[IndexX*3 + CellIndexX];
                    pSudoku->Node[IndexY][IndexX].Cell[CellIndexY][CellIndexX].pCompletedRowLookup = &pSudoku->CompletedRowLookup[IndexY*3 + CellIndexY];
                }
            }
        }
    }
}



/*******************************************************************************
 * Sudoku_NodeCellIndexesToRowMask                                                                        
 *                                                                             
 * DESCRIPTION: 
 *                                                                             
 * INPUT                                                                       
 *   
 *                                                                             
 * OUTPUT                                                                       
 *   
 *                                                                             
 ******************************************************************************/
ULONG Sudoku_NodeCellIndexesToRowMask(ULONG NodeIndexX, ULONG NodeIndexY)
{
    ULONG Mask = 0;

    Mask = (1<<(NodeIndexY*3 + 0)) | (1<<(NodeIndexY*3 + 1)) | (1<<(NodeIndexY*3 + 2));

    return Mask;
}


/*******************************************************************************
 * Sudoku_NodeCellIndexesToColMask                                                                        
 *                                                                             
 * DESCRIPTION: 
 *                                                                             
 * INPUT                                                                       
 *   
 *                                                                             
 * OUTPUT                                                                       
 *   
 *                                                                             
 ******************************************************************************/
ULONG Sudoku_NodeCellIndexesToColMask(ULONG NodeIndexX, ULONG NodeIndexY)
{
    ULONG Mask = 0;

    Mask = (1<<(0*3 + NodeIndexX)) | (1<<(1*3 + NodeIndexX)) | (1<<(2*3 + NodeIndexX));

    return Mask;
}



/*******************************************************************************
 * Sudoku_GetNodeCellLocation                                                                        
 *                                                                             
 * DESCRIPTION: 
 *                                                                             
 * INPUT                                                                       
 *   
 *                                                                             
 * OUTPUT                                                                       
 *   
 *                                                                             
 ******************************************************************************/
void Sudoku_GetNodeCellLocation(ULONG IndexX, ULONG IndexY, PNODE_CELL_LOCATION pNodeCellLocation)
{
    if(IndexY < 3)
    {
        pNodeCellLocation->NodeY = 0;
    }
    else
    {
        if(IndexY < 6)
        {
            pNodeCellLocation->NodeY = 1;
        }
        else
        {
            pNodeCellLocation->NodeY = 2;
        }
    }

    pNodeCellLocation->CellY = IndexY % 3;


    if(IndexX < 3)
    {
        pNodeCellLocation->NodeX = 0;
    }
    else
    {
        if(IndexX < 6)
        {
            pNodeCellLocation->NodeX = 1;
        }
        else
        {
            pNodeCellLocation->NodeX = 2;
        }
    }

    pNodeCellLocation->CellX = IndexX % 3;

}


/*******************************************************************************
 * Sudoku_SetBitToNumber                                                                        
 *                                                                             
 * DESCRIPTION: 
 *                                                                             
 * INPUT                                                                       
 *   
 *                                                                             
 * OUTPUT                                                                       
 *   
 *                                                                             
 ******************************************************************************/
ULONG Sudoku_SetBitToNumber(ULONG Mask)
{
    ULONG Index = 0;

    while(Mask)
    {
        Mask >>= 1;
        Index++;
    }

    return Index;
}

/*******************************************************************************
 * Sudoku_Debug_DisplayMatrix                                                                        
 *                                                                             
 * DESCRIPTION: 
 *                                                                             
 * INPUT                                                                       
 *   
 *                                                                             
 * OUTPUT                                                                       
 *   
 *                                                                             
 ******************************************************************************/
void Sudoku_Debug_DisplayMatrix(PINTERNAL_SUDOKU pSudoku)
{
    Sudoku_EnumNodesAndCellsByRows(pSudoku, Sudoku_Debug_Print, NULL);
    Sudoku_EnumNodesAndCellsByRows(pSudoku, Sudoku_Debug_PrintEx, NULL);
}

/*******************************************************************************
 * Sudoku_Debug_Print                                                                        
 *                                                                             
 * DESCRIPTION: 
 *                                                                             
 * INPUT                                                                       
 *   
 *                                                                             
 * OUTPUT                                                                       
 *   
 *                                                                             
 ******************************************************************************/
BOOL Sudoku_Debug_Print(PNODE_CELL_ENUMERATION_CTX pNodeCellEnumeration)
{

    if(pNodeCellEnumeration->pCell->Complete)
    {
        printf("%i ", pNodeCellEnumeration->pCell->Number);
    }
    else
    {
        printf("_ ");
    }
    
    if(pNodeCellEnumeration->NodeCellLocation.CellX == 2)
    {
        printf("| ");
        if(pNodeCellEnumeration->NodeCellLocation.NodeX == 2)
        {
            printf("\n");
    
            if(pNodeCellEnumeration->NodeCellLocation.CellY == 2)
            {
                printf("------------------------------\n");
        
                if(pNodeCellEnumeration->NodeCellLocation.NodeY == 2)
                {
                   printf("\n\n\n");
                }
            }
        }
    }

    return TRUE;
}

/*******************************************************************************
 * Sudoku_Debug_PrintEx                                                                        
 *                                                                             
 * DESCRIPTION: 
 *                                                                             
 * INPUT                                                                       
 *   
 *                                                                             
 * OUTPUT                                                                       
 *   
 *                                                                             
 ******************************************************************************/
BOOL Sudoku_Debug_PrintEx(PNODE_CELL_ENUMERATION_CTX pNodeCellEnumeration)
{

    printf("(0x%03x) ", pNodeCellEnumeration->pCell->PossibleMask);
    
    if(pNodeCellEnumeration->NodeCellLocation.CellX == 2)
    {
        printf("| ");
        if(pNodeCellEnumeration->NodeCellLocation.NodeX == 2)
        {
            printf("\n");
    
            if(pNodeCellEnumeration->NodeCellLocation.CellY == 2)
            {
                printf("------------------------------\n");
        
                if(pNodeCellEnumeration->NodeCellLocation.NodeY == 2)
                {
                   printf("\n\n\n");
                }
            }
        }
    }

    return TRUE;
}

/*******************************************************************************
 * Sudoku_EnumNodesAndCellsByRows                                                                        
 *                                                                             
 * DESCRIPTION: 
 *                                                                             
 * INPUT                                                                       
 *   
 *                                                                             
 * OUTPUT                                                                       
 *   
 *                                                                             
 ******************************************************************************/
void Sudoku_EnumNodesAndCellsByRows(PINTERNAL_SUDOKU pSudoku, PFN_ENUMNODESANDCELLS pEnumNodesAndCells, PVOID pContext)
{
    NODE_CELL_ENUMERATION_CTX NodeCellEnumeration = {0};
    BOOL bContinueProcessing = TRUE;

    NodeCellEnumeration.pContext = pContext;
    NodeCellEnumeration.pSudoku  = pSudoku;

    for(NodeCellEnumeration.NodeCellLocation.NodeY = 0; NodeCellEnumeration.NodeCellLocation.NodeY < 3 && bContinueProcessing; NodeCellEnumeration.NodeCellLocation.NodeY++)
    {
        for(NodeCellEnumeration.NodeCellLocation.CellY = 0; NodeCellEnumeration.NodeCellLocation.CellY < 3 && bContinueProcessing; NodeCellEnumeration.NodeCellLocation.CellY++)
        {
            for(NodeCellEnumeration.NodeCellLocation.NodeX = 0; NodeCellEnumeration.NodeCellLocation.NodeX < 3 && bContinueProcessing; NodeCellEnumeration.NodeCellLocation.NodeX++)
            {
                for(NodeCellEnumeration.NodeCellLocation.CellX = 0; NodeCellEnumeration.NodeCellLocation.CellX < 3 && bContinueProcessing; NodeCellEnumeration.NodeCellLocation.CellX++)
                {
                    NodeCellEnumeration.pNode = &pSudoku->Node[NodeCellEnumeration.NodeCellLocation.NodeY][NodeCellEnumeration.NodeCellLocation.NodeX];
                    NodeCellEnumeration.pCell = &NodeCellEnumeration.pNode->Cell[NodeCellEnumeration.NodeCellLocation.CellY][NodeCellEnumeration.NodeCellLocation.CellX];

                    bContinueProcessing = pEnumNodesAndCells(&NodeCellEnumeration);
                }
            }
        }
    }
}

/*******************************************************************************
 * Sudoku_EnumNodesAndCellsByCols                                                                        
 *                                                                             
 * DESCRIPTION: 
 *                                                                             
 * INPUT                                                                       
 *   
 *                                                                             
 * OUTPUT                                                                       
 *   
 *                                                                             
 ******************************************************************************/
void Sudoku_EnumNodesAndCellsByCols(PINTERNAL_SUDOKU pSudoku, PFN_ENUMNODESANDCELLS pEnumNodesAndCells, PVOID pContext)
{
    NODE_CELL_ENUMERATION_CTX NodeCellEnumeration = {0};
    BOOL bContinueProcessing = TRUE;

    NodeCellEnumeration.pContext = pContext;
    NodeCellEnumeration.pSudoku  = pSudoku;

    for(NodeCellEnumeration.NodeCellLocation.NodeX = 0; NodeCellEnumeration.NodeCellLocation.NodeX < 3 && bContinueProcessing; NodeCellEnumeration.NodeCellLocation.NodeX++)
    {
        for(NodeCellEnumeration.NodeCellLocation.CellX = 0; NodeCellEnumeration.NodeCellLocation.CellX < 3 && bContinueProcessing; NodeCellEnumeration.NodeCellLocation.CellX++)
        {
            for(NodeCellEnumeration.NodeCellLocation.NodeY = 0; NodeCellEnumeration.NodeCellLocation.NodeY < 3 && bContinueProcessing; NodeCellEnumeration.NodeCellLocation.NodeY++)
            {
                for(NodeCellEnumeration.NodeCellLocation.CellY = 0; NodeCellEnumeration.NodeCellLocation.CellY < 3 && bContinueProcessing; NodeCellEnumeration.NodeCellLocation.CellY++)
                {
                    NodeCellEnumeration.pNode = &pSudoku->Node[NodeCellEnumeration.NodeCellLocation.NodeY][NodeCellEnumeration.NodeCellLocation.NodeX];
                    NodeCellEnumeration.pCell = &NodeCellEnumeration.pNode->Cell[NodeCellEnumeration.NodeCellLocation.CellY][NodeCellEnumeration.NodeCellLocation.CellX];

                    bContinueProcessing = pEnumNodesAndCells(&NodeCellEnumeration);
                }
            }
        }
    }
}


/*******************************************************************************
 * Sudoku_EnumNodesAndCellsByNodes                                                                        
 *                                                                             
 * DESCRIPTION: 
 *                                                                             
 * INPUT                                                                       
 *   
 *                                                                             
 * OUTPUT                                                                       
 *   
 *                                                                             
 ******************************************************************************/
void Sudoku_EnumNodesAndCellsByNodes(PINTERNAL_SUDOKU pSudoku, PFN_ENUMNODESANDCELLS pEnumNodesAndCells, PVOID pContext, ULONG Flags)
{
    NODE_CELL_ENUMERATION_CTX NodeCellEnumeration = {0};
    BOOL bContinueProcessing = TRUE;

    NodeCellEnumeration.pContext = pContext;
    NodeCellEnumeration.pSudoku  = pSudoku;

    
    for(NodeCellEnumeration.NodeCellLocation.NodeY = 0; NodeCellEnumeration.NodeCellLocation.NodeY < 3 && bContinueProcessing; NodeCellEnumeration.NodeCellLocation.NodeY++)
    {
        for(NodeCellEnumeration.NodeCellLocation.NodeX = 0; NodeCellEnumeration.NodeCellLocation.NodeX < 3 && bContinueProcessing; NodeCellEnumeration.NodeCellLocation.NodeX++)
        {
            NodeCellEnumeration.pNode = &pSudoku->Node[NodeCellEnumeration.NodeCellLocation.NodeY][NodeCellEnumeration.NodeCellLocation.NodeX];

            if(NodeCellEnumeration.pNode->Complete == FALSE || (Flags & FLAG_SKIP_COMPLETE_NODES) == 0)
            {
                for(NodeCellEnumeration.NodeCellLocation.CellY = 0; NodeCellEnumeration.NodeCellLocation.CellY < 3 && bContinueProcessing; NodeCellEnumeration.NodeCellLocation.CellY++)
                {
                    for(NodeCellEnumeration.NodeCellLocation.CellX = 0; NodeCellEnumeration.NodeCellLocation.CellX < 3 && bContinueProcessing; NodeCellEnumeration.NodeCellLocation.CellX++)
                    {
                        NodeCellEnumeration.pCell = &NodeCellEnumeration.pNode->Cell[NodeCellEnumeration.NodeCellLocation.CellY][NodeCellEnumeration.NodeCellLocation.CellX];
    
                        bContinueProcessing = pEnumNodesAndCells(&NodeCellEnumeration);
                    }
                }
            }
        }
    }
}

/*******************************************************************************
 * Sudoku_EnumCellsByNode                                                                        
 *                                                                             
 * DESCRIPTION: 
 *                                                                             
 * INPUT                                                                       
 *   
 *                                                                             
 * OUTPUT                                                                       
 *   
 *                                                                             
 ******************************************************************************/
void Sudoku_EnumCellsByNode(PINTERNAL_SUDOKU pSudoku, PSUDOKU_NODE pNode, PFN_ENUMNODESANDCELLS pEnumNodesAndCells, PVOID pContext)
{
    NODE_CELL_ENUMERATION_CTX NodeCellEnumeration = {0};
    BOOL bContinueProcessing = TRUE;

    NodeCellEnumeration.pContext = pContext;
    NodeCellEnumeration.pSudoku  = pSudoku;
    NodeCellEnumeration.pNode    = pNode;

    for(NodeCellEnumeration.NodeCellLocation.CellY = 0; NodeCellEnumeration.NodeCellLocation.CellY < 3 && bContinueProcessing; NodeCellEnumeration.NodeCellLocation.CellY++)
    {
        for(NodeCellEnumeration.NodeCellLocation.CellX = 0; NodeCellEnumeration.NodeCellLocation.CellX < 3 && bContinueProcessing; NodeCellEnumeration.NodeCellLocation.CellX++)
        {
            NodeCellEnumeration.pCell = &NodeCellEnumeration.pNode->Cell[NodeCellEnumeration.NodeCellLocation.CellY][NodeCellEnumeration.NodeCellLocation.CellX];

            bContinueProcessing = pEnumNodesAndCells(&NodeCellEnumeration);
        }
    }
}

/*******************************************************************************
 * Sudoku_CreateGlobalSyncContext                                                                        
 *                                                                             
 * DESCRIPTION: 
 *                                                                             
 * INPUT                                                                       
 *   
 *                                                                             
 * OUTPUT                                                                       
 *   
 *                                                                             
 ******************************************************************************/
PSUDOKU_GLOBAL_SYNC Sudoku_CreateGlobalSyncContext(void)
{
    PSUDOKU_GLOBAL_SYNC pSudokuEngine;

    pSudokuEngine = (PSUDOKU_GLOBAL_SYNC)LocalAlloc(LMEM_ZEROINIT, sizeof(SUDOKU_GLOBAL_SYNC));

    if(pSudokuEngine)
    {
        InitializeCriticalSection(&pSudokuEngine->csEngineLock);

        pSudokuEngine->hEngineComplete = CreateEvent(NULL, FALSE, FALSE, NULL);
        pSudokuEngine->hWorkAvailable = CreateEvent(NULL, TRUE, FALSE, NULL);
    }

    return pSudokuEngine;

}


/*******************************************************************************
 * Sudoku_ProcessVerifyConsistency                                                                        
 *                                                                             
 * DESCRIPTION: 
 *                                                                             
 * INPUT                                                                       
 *   
 *                                                                             
 * OUTPUT                                                                       
 *   
 *                                                                             
 ******************************************************************************/
void Sudoku_ProcessVerifyConsistency(PINTERNAL_SUDOKU pSudoku)
{
    SUDOKU_CONSISTENCY SudokuConsistency = {0};
    UINT Index;
    UINT NodeX;
    UINT NodeY;

    Sudoku_EnumNodesAndCellsByRows(pSudoku, Sudoku_Consistency_Callback, &SudokuConsistency);

    for(Index = 0; Index < 9; Index++)
    {
        if(pSudoku->CompletedColLookup[Index] != SudokuConsistency.ColComplete[Index])
        {
            Sudoku_Debug("Inconsistent State: Completed Column Tracking %i\n", Index);
            Sudoku_SetNextState(pSudoku, SudokuStateFailure);
        }

        if(pSudoku->CompletedRowLookup[Index] != SudokuConsistency.RowComplete[Index])
        {
            Sudoku_Debug("Inconsistent State: Completed Row Tracking %i\n", Index);
            Sudoku_SetNextState(pSudoku, SudokuStateFailure);
        }

        if((SudokuConsistency.ColComplete[Index] ^ SudokuConsistency.ColPossible[Index]) != SUDOKU_COMPLETE_MASK)
        {
            Sudoku_Debug("Inconsistent State: Column %i (0x%0x ^ 0x%0x)\n", Index, SudokuConsistency.ColComplete[Index], SudokuConsistency.ColPossible[Index]);
            Sudoku_SetNextState(pSudoku, SudokuStateFailure);
        }

        if((SudokuConsistency.RowComplete[Index] ^ SudokuConsistency.RowPossible[Index]) != SUDOKU_COMPLETE_MASK)
        {
            Sudoku_Debug("Inconsistent State: Row %i (0x%0x ^ 0x%0x)\n", Index, SudokuConsistency.ColComplete[Index], SudokuConsistency.ColPossible[Index]);
            Sudoku_SetNextState(pSudoku, SudokuStateFailure);
        }

    }

    for(NodeY = 0; NodeY < 3; NodeY++)
    {
        for(NodeX = 0; NodeX < 3; NodeX++)
        {
            if((SudokuConsistency.NodeCompleteMatrix[NodeY][NodeX] ^ SudokuConsistency.NodePossibleMatrix[NodeY][NodeX]) != SUDOKU_COMPLETE_MASK)
            {
                    Sudoku_Debug("Inconsistent State: Node %i, %i (0x%0x, 0x%0x)\n", NodeX, NodeY, SudokuConsistency.NodeCompleteMatrix[NodeY][NodeX], SudokuConsistency.NodePossibleMatrix[NodeY][NodeX]);
                    Sudoku_SetNextState(pSudoku, SudokuStateFailure);
            }
        }
    }

}


/*******************************************************************************
 * Sudoku_ProcessSuperCondenseRows                                                                        
 *                                                                             
 * DESCRIPTION: 
 *                                                                             
 * INPUT                                                                       
 *   
 *                                                                             
 * OUTPUT                                                                       
 *   
 *                                                                             
 ******************************************************************************/
void Sudoku_ProcessSuperCondenseRows(PINTERNAL_SUDOKU pSudoku)
{
    UINT RowIndex;
    UINT CellYIndex;
    UINT NodeYIndex;
    UINT NodeIndex;
    ULONG PossibleMasks[3];
    ULONG Combinations[5];

    NodeYIndex = -1;

    for(RowIndex = 0; RowIndex < 9; RowIndex++)
    {
        CellYIndex = RowIndex % 3;

        if(CellYIndex == 0)
        {
            NodeYIndex++;
        }

        for(NodeIndex = 0; NodeIndex < 3; NodeIndex++)
        {
            PossibleMasks[0] = 0;
            PossibleMasks[1] = 0;
            PossibleMasks[2] = 0;

            if(pSudoku->Node[NodeYIndex][NodeIndex].Complete == FALSE)
            {
                if(pSudoku->Node[NodeYIndex][NodeIndex].Cell[CellYIndex][0].Complete == FALSE)
                {
                    PossibleMasks[0] = pSudoku->Node[NodeYIndex][NodeIndex].Cell[CellYIndex][0].PossibleMask;
                }

                if(pSudoku->Node[NodeYIndex][NodeIndex].Cell[CellYIndex][1].Complete == FALSE)
                {
                     PossibleMasks[1] = pSudoku->Node[NodeYIndex][NodeIndex].Cell[CellYIndex][1].PossibleMask;
                }

                if(pSudoku->Node[NodeYIndex][NodeIndex].Cell[CellYIndex][2].Complete == FALSE)
                {
                    PossibleMasks[2] = pSudoku->Node[NodeYIndex][NodeIndex].Cell[CellYIndex][2].PossibleMask;
                }

                Combinations[0] = PossibleMasks[0] & PossibleMasks[1] & PossibleMasks[2];
                Combinations[1] = PossibleMasks[0] & PossibleMasks[1];
                Combinations[2] = PossibleMasks[0] & PossibleMasks[2];
                Combinations[3] = PossibleMasks[1] & PossibleMasks[2];

                Combinations[4] = CellYIndex;

                if(Combinations[0] || Combinations[1] || Combinations[2] || Combinations[3])
                {
                    Sudoku_EnumCellsByNode(pSudoku, &pSudoku->Node[NodeYIndex][NodeIndex], Sudoku_ProcessSuperCondenseRows_CompareCells_Callback, (PVOID)&Combinations[0]);


                    if(PossibleMasks[0] == Combinations[0] && 
                       PossibleMasks[1] == Combinations[0] && 
                       PossibleMasks[2] == Combinations[0])
                    {
                        Combinations[0] = 0;
                    }

                    if(PossibleMasks[0] == Combinations[1] && 
                       PossibleMasks[1] == Combinations[1])
                    {
                        Combinations[1] = 0;
                    }

                    if(PossibleMasks[0] == Combinations[2] && 
                       PossibleMasks[2] == Combinations[2])
                    {
                        Combinations[2] = 0;
                    }

                    if(PossibleMasks[1] == Combinations[3] && 
                       PossibleMasks[2] == Combinations[3])
                    {
                        Combinations[3] = 0;
                    }


                    if(Sudoku_NumerOfBits(Combinations[0]) == 3)
                    {
                        Sudoku_MaskCellPossibilities(pSudoku, &pSudoku->Node[NodeYIndex][NodeIndex].Cell[CellYIndex][0], Combinations[0]);
                        Sudoku_MaskCellPossibilities(pSudoku, &pSudoku->Node[NodeYIndex][NodeIndex].Cell[CellYIndex][1], Combinations[0]);
                        Sudoku_MaskCellPossibilities(pSudoku, &pSudoku->Node[NodeYIndex][NodeIndex].Cell[CellYIndex][2], Combinations[0]);
                    }

                    if(Sudoku_NumerOfBits(Combinations[1]) == 2 && (PossibleMasks[2] & Combinations[1]) == 0)
                    {
                        Sudoku_MaskCellPossibilities(pSudoku, &pSudoku->Node[NodeYIndex][NodeIndex].Cell[CellYIndex][0], Combinations[1]);
                        Sudoku_MaskCellPossibilities(pSudoku, &pSudoku->Node[NodeYIndex][NodeIndex].Cell[CellYIndex][1], Combinations[1]);
                    }

                    if(Sudoku_NumerOfBits(Combinations[2]) == 2 && (PossibleMasks[1] & Combinations[2]) == 0)
                    {
                        Sudoku_MaskCellPossibilities(pSudoku, &pSudoku->Node[NodeYIndex][NodeIndex].Cell[CellYIndex][0], Combinations[2]);
                        Sudoku_MaskCellPossibilities(pSudoku, &pSudoku->Node[NodeYIndex][NodeIndex].Cell[CellYIndex][2], Combinations[2]);
                    }

                    if(Sudoku_NumerOfBits(Combinations[3]) == 2 && (PossibleMasks[0] & Combinations[3]) == 0)
                    {
                        Sudoku_MaskCellPossibilities(pSudoku, &pSudoku->Node[NodeYIndex][NodeIndex].Cell[CellYIndex][1], Combinations[3]);
                        Sudoku_MaskCellPossibilities(pSudoku, &pSudoku->Node[NodeYIndex][NodeIndex].Cell[CellYIndex][2], Combinations[3]);
                    }
                }
            }
        }
    }
}


/*******************************************************************************
 * Sudoku_ProcessSuperCondenseRows_CompareCells_Callback                                                                        
 *                                                                             
 * DESCRIPTION: 
 *                                                                             
 * INPUT                                                                       
 *   
 *                                                                             
 * OUTPUT                                                                       
 *   
 *                                                                             
 ******************************************************************************/
BOOL Sudoku_ProcessSuperCondenseRows_CompareCells_Callback(PNODE_CELL_ENUMERATION_CTX pNodeCellEnumeration)
{
    BOOL ContinueProcessing = TRUE;
    ULONG *pCondensedMasks;

    pCondensedMasks = ((ULONG *)pNodeCellEnumeration->pContext);

    if(pNodeCellEnumeration->pCell->Complete == FALSE)
    {
        if(pNodeCellEnumeration->NodeCellLocation.CellY != pCondensedMasks[4])
        {
            pCondensedMasks[0] = (pCondensedMasks[0] ^ pNodeCellEnumeration->pCell->PossibleMask) & pCondensedMasks[0];
            pCondensedMasks[1] = (pCondensedMasks[1] ^ pNodeCellEnumeration->pCell->PossibleMask) & pCondensedMasks[1];
            pCondensedMasks[2] = (pCondensedMasks[2] ^ pNodeCellEnumeration->pCell->PossibleMask) & pCondensedMasks[2];
            pCondensedMasks[3] = (pCondensedMasks[3] ^ pNodeCellEnumeration->pCell->PossibleMask) & pCondensedMasks[3];

            if(pCondensedMasks[0] == 0 && pCondensedMasks[1] == 0 && pCondensedMasks[2] == 0 && pCondensedMasks[3] == 0)
            {
                ContinueProcessing = FALSE;
            }
        }
    }

    return ContinueProcessing;
}


/*******************************************************************************
 * Sudoku_ProcessSuperCondenseCols_CompareCells_Callback                                                                        
 *                                                                             
 * DESCRIPTION: 
 *                                                                             
 * INPUT                                                                       
 *   
 *                                                                             
 * OUTPUT                                                                       
 *   
 *                                                                             
 ******************************************************************************/
BOOL Sudoku_ProcessSuperCondenseCols_CompareCells_Callback(PNODE_CELL_ENUMERATION_CTX pNodeCellEnumeration)
{
    BOOL ContinueProcessing = TRUE;
    ULONG *pCondensedMasks;

    pCondensedMasks = ((ULONG *)pNodeCellEnumeration->pContext);

    if(pNodeCellEnumeration->pCell->Complete == FALSE)
    {
        if(pNodeCellEnumeration->NodeCellLocation.CellX != pCondensedMasks[4])
        {
            pCondensedMasks[0] = (pCondensedMasks[0] ^ pNodeCellEnumeration->pCell->PossibleMask) & pCondensedMasks[0];
            pCondensedMasks[1] = (pCondensedMasks[1] ^ pNodeCellEnumeration->pCell->PossibleMask) & pCondensedMasks[1];
            pCondensedMasks[2] = (pCondensedMasks[2] ^ pNodeCellEnumeration->pCell->PossibleMask) & pCondensedMasks[2];
            pCondensedMasks[3] = (pCondensedMasks[3] ^ pNodeCellEnumeration->pCell->PossibleMask) & pCondensedMasks[3];

            if(pCondensedMasks[0] == 0 && pCondensedMasks[1] == 0 && pCondensedMasks[2] == 0 && pCondensedMasks[3] == 0)
            {
                ContinueProcessing = FALSE;
            }
        }
    }

    return ContinueProcessing;
}



/*******************************************************************************
 * Sudoku_ProcessSuperCondenseCols                                                                        
 *                                                                             
 * DESCRIPTION: 
 *                                                                             
 * INPUT                                                                       
 *   
 *                                                                             
 * OUTPUT                                                                       
 *   
 *                                                                             
 ******************************************************************************/
void Sudoku_ProcessSuperCondenseCols(PINTERNAL_SUDOKU pSudoku)
{
    UINT ColIndex;
    UINT CellXIndex;
    UINT NodeXIndex;
    UINT NodeIndex;
    ULONG PossibleMasks[3];
    ULONG Combinations[5];

    NodeXIndex = -1;

    for(ColIndex = 0; ColIndex < 9; ColIndex++)
    {
        CellXIndex = ColIndex % 3;

        if(CellXIndex == 0)
        {
            NodeXIndex++;
        }

        for(NodeIndex = 0; NodeIndex < 3; NodeIndex++)
        {
            PossibleMasks[0] = 0;
            PossibleMasks[1] = 0;
            PossibleMasks[2] = 0;

            if(pSudoku->Node[NodeIndex][NodeXIndex].Complete == FALSE)
            {
                if(pSudoku->Node[NodeIndex][NodeXIndex].Cell[0][CellXIndex].Complete == FALSE)
                {
                    PossibleMasks[0] = pSudoku->Node[NodeIndex][NodeXIndex].Cell[0][CellXIndex].PossibleMask;
                }

                if(pSudoku->Node[NodeIndex][NodeXIndex].Cell[1][CellXIndex].Complete == FALSE)
                {
                     PossibleMasks[1] = pSudoku->Node[NodeIndex][NodeXIndex].Cell[1][CellXIndex].PossibleMask;
                }

                if(pSudoku->Node[NodeIndex][NodeXIndex].Cell[2][CellXIndex].Complete == FALSE)
                {
                    PossibleMasks[2] = pSudoku->Node[NodeIndex][NodeXIndex].Cell[2][CellXIndex].PossibleMask;
                }

                Combinations[0] = PossibleMasks[0] & PossibleMasks[1] & PossibleMasks[2];
                Combinations[1] = PossibleMasks[0] & PossibleMasks[1];
                Combinations[2] = PossibleMasks[0] & PossibleMasks[2];
                Combinations[3] = PossibleMasks[1] & PossibleMasks[2];

                Combinations[4] = CellXIndex;

                if(Combinations[0] || Combinations[1] || Combinations[2] || Combinations[3])
                {
                    Sudoku_EnumCellsByNode(pSudoku, &pSudoku->Node[NodeIndex][NodeXIndex], Sudoku_ProcessSuperCondenseCols_CompareCells_Callback, (PVOID)&Combinations[0]);

                    if(PossibleMasks[0] == Combinations[0] && 
                       PossibleMasks[1] == Combinations[0] && 
                       PossibleMasks[2] == Combinations[0])
                    {
                        Combinations[0] = 0;
                    }

                    if(PossibleMasks[0] == Combinations[1] && 
                       PossibleMasks[1] == Combinations[1])
                    {
                        Combinations[1] = 0;
                    }

                    if(PossibleMasks[0] == Combinations[2] && 
                       PossibleMasks[2] == Combinations[2])
                    {
                        Combinations[2] = 0;
                    }

                    if(PossibleMasks[1] == Combinations[3] && 
                       PossibleMasks[2] == Combinations[3])
                    {
                        Combinations[3] = 0;
                    }


                    if(Sudoku_NumerOfBits(Combinations[0]) == 3)
                    {
                        Sudoku_MaskCellPossibilities(pSudoku, &pSudoku->Node[NodeIndex][NodeXIndex].Cell[0][CellXIndex], Combinations[0]);
                        Sudoku_MaskCellPossibilities(pSudoku, &pSudoku->Node[NodeIndex][NodeXIndex].Cell[1][CellXIndex], Combinations[0]);
                        Sudoku_MaskCellPossibilities(pSudoku, &pSudoku->Node[NodeIndex][NodeXIndex].Cell[2][CellXIndex], Combinations[0]);
                    }

                    if(Sudoku_NumerOfBits(Combinations[1]) == 2 && (PossibleMasks[2] & Combinations[1]) == 0)
                    {
                        Sudoku_MaskCellPossibilities(pSudoku, &pSudoku->Node[NodeIndex][NodeXIndex].Cell[0][CellXIndex], Combinations[1]);
                        Sudoku_MaskCellPossibilities(pSudoku, &pSudoku->Node[NodeIndex][NodeXIndex].Cell[1][CellXIndex], Combinations[1]);
                    }

                    if(Sudoku_NumerOfBits(Combinations[2]) == 2 && (PossibleMasks[1] & Combinations[2]) == 0)
                    {
                        Sudoku_MaskCellPossibilities(pSudoku, &pSudoku->Node[NodeIndex][NodeXIndex].Cell[0][CellXIndex], Combinations[2]);
                        Sudoku_MaskCellPossibilities(pSudoku, &pSudoku->Node[NodeIndex][NodeXIndex].Cell[2][CellXIndex], Combinations[2]);
                    }

                    if(Sudoku_NumerOfBits(Combinations[3]) == 2 && (PossibleMasks[0] & Combinations[3]) == 0)
                    {
                        Sudoku_MaskCellPossibilities(pSudoku, &pSudoku->Node[NodeIndex][NodeXIndex].Cell[1][CellXIndex], Combinations[3]);
                        Sudoku_MaskCellPossibilities(pSudoku, &pSudoku->Node[NodeIndex][NodeXIndex].Cell[2][CellXIndex], Combinations[3]);
                    }
                }
            }
        }
    }

}



/*******************************************************************************
 * Sudoku_Consistency_Callback                                                                        
 *                                                                             
 * DESCRIPTION: 
 *                                                                             
 * INPUT                                                                       
 *   
 *                                                                             
 * OUTPUT                                                                       
 *   
 *                                                                             
 ******************************************************************************/
BOOL Sudoku_Consistency_Callback(PNODE_CELL_ENUMERATION_CTX pNodeCellEnumeration)
{
    PSUDOKU_CONSISTENCY pSudokuConsistency = (PSUDOKU_CONSISTENCY)pNodeCellEnumeration->pContext;
    ULONG NodeX;
    ULONG NodeY;
    ULONG CellX;
    ULONG CellY;
    
    CellY = pNodeCellEnumeration->NodeCellLocation.CellY;
    CellX = pNodeCellEnumeration->NodeCellLocation.CellX;
    NodeX = pNodeCellEnumeration->NodeCellLocation.NodeX;
    NodeY = pNodeCellEnumeration->NodeCellLocation.NodeY;

    if(pNodeCellEnumeration->pCell->Complete)
    {
        if((pSudokuConsistency->NodeCompleteMatrix[NodeY][NodeX] & pNodeCellEnumeration->pCell->PossibleMask) != 0)
        {
            Sudoku_Debug("Inconsistent State: Node %i, %i Cell %i, %i  Invalid Possibility Mask In Node Complete Tracking\n", NodeX, NodeY, CellX, CellY);
            Sudoku_SetNextState(pNodeCellEnumeration->pSudoku, SudokuStateFailure);
        }
        else
        {
            pSudokuConsistency->NodeCompleteMatrix[NodeY][NodeX] |= pNodeCellEnumeration->pCell->PossibleMask;
        }

        if((pSudokuConsistency->RowComplete[NodeY*3 + CellY] & pNodeCellEnumeration->pCell->PossibleMask) != 0)
        {
            Sudoku_Debug("Inconsistent State: Node %i, %i Cell %i, %i  Invalid Possibility Mask In Row Complete Tracking\n", NodeX, NodeY, CellX, CellY);
            Sudoku_SetNextState(pNodeCellEnumeration->pSudoku, SudokuStateFailure);
        }
        else
        {
            pSudokuConsistency->RowComplete[NodeY*3 + CellY] |= pNodeCellEnumeration->pCell->PossibleMask;
        }

        if((pSudokuConsistency->ColComplete[NodeX*3 + CellX] & pNodeCellEnumeration->pCell->PossibleMask) != 0)
        {
            Sudoku_Debug("Inconsistent State: Node %i, %i Cell %i, %i  Invalid Possibility Mask In Col Complete Tracking\n", NodeX, NodeY, CellX, CellY);
            Sudoku_SetNextState(pNodeCellEnumeration->pSudoku, SudokuStateFailure);
        }
        else
        {
            pSudokuConsistency->ColComplete[NodeX*3 + CellX] |= pNodeCellEnumeration->pCell->PossibleMask;
        }
    }
    else
    {
        pSudokuConsistency->NodePossibleMatrix[NodeY][NodeX] |= pNodeCellEnumeration->pCell->PossibleMask;
        pSudokuConsistency->RowPossible[NodeY*3 + CellY]     |= pNodeCellEnumeration->pCell->PossibleMask;
        pSudokuConsistency->ColPossible[NodeX*3 + CellX]     |= pNodeCellEnumeration->pCell->PossibleMask;

        if(pNodeCellEnumeration->pCell->PossibleMask == 0)
        {
            Sudoku_SetNextState(pNodeCellEnumeration->pSudoku, SudokuStateBug);
        }
    }

    return TRUE;
}


/*******************************************************************************
 * Sudoku_ProcessCondenseNodeCells                                                                        
 *                                                                             
 * DESCRIPTION: 
 *                                                                             
 * INPUT                                                                       
 *   
 *                                                                             
 * OUTPUT                                                                       
 *   
 *                                                                             
 ******************************************************************************/
void Sudoku_ProcessCondenseNodeCells(PINTERNAL_SUDOKU pSudoku)
{
    // TBD Condense where 2 cells contains only 2 possiblities, remove the rest from the rest of the cells.
    Sudoku_EnumNodesAndCellsByNodes(pSudoku, Sudoku_ProcessCondenseNodeCells_Callback, NULL, FLAG_SKIP_COMPLETE_NODES);
}

/*******************************************************************************
 * Sudoku_ProcessSuperCondenseNodeCells                                                                        
 *                                                                             
 * DESCRIPTION: 
 *                                                                             
 * INPUT                                                                       
 *   
 *                                                                             
 * OUTPUT                                                                       
 *   
 *                                                                             
 ******************************************************************************/
void Sudoku_ProcessSuperCondenseNodeCells(PINTERNAL_SUDOKU pSudoku)
{
    // TBD Super Condense where 2 Cells contain an intersection of 2 numbers not found in other cells, remove the rest of the possible numbers.
    Sudoku_EnumNodesAndCellsByNodes(pSudoku, Sudoku_ProcessSuperCondenseNodeCells_Callback, NULL, FLAG_SKIP_COMPLETE_NODES);
}


/*******************************************************************************
 * Sudoku_ProcessCondenseNodeCells_Callback                                                                        
 *                                                                             
 * DESCRIPTION: 
 *                                                                             
 * INPUT                                                                       
 *   
 *                                                                             
 * OUTPUT                                                                       
 *   
 *                                                                             
 ******************************************************************************/
BOOL Sudoku_ProcessCondenseNodeCells_Callback(PNODE_CELL_ENUMERATION_CTX pNodeCellEnumeration)
{
     
    return TRUE;
}


/*******************************************************************************
 * Sudoku_ProcessSuperCondenseNodeCells_Callback                                                                        
 *                                                                             
 * DESCRIPTION: 
 *                                                                             
 * INPUT                                                                       
 *   
 *                                                                             
 * OUTPUT                                                                       
 *   
 *                                                                             
 ******************************************************************************/
BOOL Sudoku_ProcessSuperCondenseNodeCells_Callback(PNODE_CELL_ENUMERATION_CTX pNodeCellEnumeration)
{
    return TRUE;
}


/*******************************************************************************
 * Sudoku_ProcessFailure                                                                        
 *                                                                             
 * DESCRIPTION: 
 *                                                                             
 * INPUT                                                                       
 *   
 *                                                                             
 * OUTPUT                                                                       
 *   
 *                                                                             
 ******************************************************************************/
void Sudoku_ProcessFailure(PINTERNAL_SUDOKU pSudoku)
{
    Sudoku_Debug("Inconsistent State\n");
    Sudoku_UpdateCounter(pSudoku, CounterTypeFailure, 1);
    Sudoku_StopEngine(pSudoku);
}



/*******************************************************************************
 * Sudoku_NumerOfBits                                                                        
 *                                                                             
 * DESCRIPTION: 
 *                                                                             
 * INPUT                                                                       
 *   
 *                                                                             
 * OUTPUT                                                                       
 *   
 *                                                                             
 ******************************************************************************/
ULONG Sudoku_NumerOfBits(ULONG BitMask)
{
    ULONG NumberOfBits = 0;

    while(BitMask)
    {
        if(BitMask & 1)
        {
            NumberOfBits++;
        }

        BitMask >>= 1;    
    }

    return NumberOfBits;
}



/*******************************************************************************
 * Sudoku_CreateDuplicateContext                                                                        
 *                                                                             
 * DESCRIPTION: 
 *                                                                             
 * INPUT                                                                       
 *   
 *                                                                             
 * OUTPUT                                                                       
 *   
 *                                                                             
 ******************************************************************************/
PINTERNAL_SUDOKU Sudoku_CreateDuplicateContext(PINTERNAL_SUDOKU pSudoku)
{
    PINTERNAL_SUDOKU pInternalSudoku;
    ULONG IndexX;
    ULONG IndexY;
    ULONG CellX;
    ULONG CellY;
    PSUDOKU_CELL pNewCell;
    PSUDOKU_CELL pOriginalCell;

    pInternalSudoku = (PINTERNAL_SUDOKU)LocalAlloc(LMEM_ZEROINIT, sizeof(INTERNAL_SUDOKU));

    if(pInternalSudoku)
    {
        Sudoku_InitializeNodesAndCells(pInternalSudoku);

        memcpy(pInternalSudoku->CompletedColLookup, pSudoku->CompletedColLookup, sizeof(pSudoku->CompletedColLookup));
        memcpy(pInternalSudoku->CompletedRowLookup, pSudoku->CompletedRowLookup, sizeof(pSudoku->CompletedRowLookup));
        pInternalSudoku->CompleteNodesBitmask = pSudoku->CompleteNodesBitmask;

        pInternalSudoku->NumberOfCompletedNodes = pSudoku->NumberOfCompletedNodes;
        pInternalSudoku->NumberOfCompletedRows = pSudoku->NumberOfCompletedRows;
        pInternalSudoku->NumberOfCompletedCols = pSudoku->NumberOfCompletedCols;
        pInternalSudoku->Priority = pSudoku->Priority;

        pInternalSudoku->Flags = pSudoku->Flags;
        pInternalSudoku->DirtyNodesBitmask = pSudoku->DirtyNodesBitmask;
        pInternalSudoku->pSudokuGlobalSync = pSudoku->pSudokuGlobalSync;

        for(IndexY = 0; IndexY < 3; IndexY++)
        {
            for(IndexX = 0; IndexX < 3; IndexX++)
            {
                pInternalSudoku->Node[IndexY][IndexX].Complete = pSudoku->Node[IndexY][IndexX].Complete;
                pInternalSudoku->Node[IndexY][IndexX].CompletedMask = pSudoku->Node[IndexY][IndexX].CompletedMask;
                pInternalSudoku->Node[IndexY][IndexX].NodeBitmask = pSudoku->Node[IndexY][IndexX].NodeBitmask;

                for(CellY = 0; CellY < 3; CellY++)
                {
                    for(CellX = 0; CellX < 3; CellX++)
                    {
                        pOriginalCell = &pSudoku->Node[IndexY][IndexX].Cell[CellY][CellX];
                        pNewCell = &pInternalSudoku->Node[IndexY][IndexX].Cell[CellY][CellX];

                        pNewCell->Complete = pOriginalCell->Complete;
                        pNewCell->NodesDirtyMask = pOriginalCell->NodesDirtyMask;
                        pNewCell->Number = pOriginalCell->Number;
                        pNewCell->PossibleMask = pOriginalCell->PossibleMask;
                    }
                }
            }
        }

        pInternalSudoku->State = SudokuStateProcessDirtyNodes;
        pInternalSudoku->NextState = SudokuStateVerifyConsistency;

    }
    
    return (HSUDOKU)pInternalSudoku;
}


/*******************************************************************************
 * Sudoku_SplitEngineWorkerThread                                                                        
 *                                                                             
 * DESCRIPTION: 
 *                                                                             
 * INPUT                                                                       
 *   
 *                                                                             
 * OUTPUT                                                                       
 *   
 *                                                                             
 ******************************************************************************/
DWORD Sudoku_SplitEngineWorkerThread(PVOID pEngineContext)
{
    PSUDOKU_GLOBAL_SYNC pEngineThreadContext = (PSUDOKU_GLOBAL_SYNC)pEngineContext;
    PINTERNAL_SUDOKU pSudoku;

    while(pEngineThreadContext->StopAllEngines == FALSE)
    {
        EnterCriticalSection(&pEngineThreadContext->csEngineLock);

        if(Sudoku_WorkerQueuesAreEmpty(pEngineThreadContext, 0) == FALSE && pEngineThreadContext->StopAllEngines == FALSE)
        {
            pSudoku = Sudoku_GetNextWorkItem(pEngineThreadContext, 0);

            if(Sudoku_WorkerQueuesAreEmpty(pEngineThreadContext, 0))
            {
                ResetEvent(pEngineThreadContext->hWorkAvailable);
            }

            pEngineThreadContext->ConcurrentThreads++;

            LeaveCriticalSection(&pEngineThreadContext->csEngineLock);
            pSudoku = Sudoku_StateEngine(pSudoku);
            EnterCriticalSection(&pEngineThreadContext->csEngineLock);

            pEngineThreadContext->ConcurrentThreads--;

            Sudoku_PreserveTotalEngineCounters(pSudoku);

            if(pSudoku->State == SudokuStateSolved)
            {
                if(pSudoku->Flags & SF_SOLVE_ALL_PATHS)
                {
                    pSudoku->pSolutionQueueNext = pEngineThreadContext->pSuccessConfiguration;
                    pEngineThreadContext->pSuccessConfiguration = pSudoku;
                }
                else
                {
                    if(pEngineThreadContext->pSuccessConfiguration == NULL)
                    {     
                        pEngineThreadContext->pSuccessConfiguration = pSudoku;
                        SetEvent(pEngineThreadContext->hWorkAvailable);
                        SetEvent(pEngineThreadContext->hEngineComplete);
                    }
                    else
                    {
                        LocalFree(pSudoku);
                    }
                }
            }
            else
            {
                LocalFree(pSudoku);
            }

            if(pEngineThreadContext->ConcurrentThreads == 0 && 
               Sudoku_WorkerQueuesAreEmpty(pEngineThreadContext, 0) && 
               pEngineThreadContext->StopAllEngines == FALSE)
            {
                pEngineThreadContext->StopAllEngines = TRUE;
                SetEvent(pEngineThreadContext->hWorkAvailable);
                SetEvent(pEngineThreadContext->hEngineComplete);
            }
        }

        LeaveCriticalSection(&pEngineThreadContext->csEngineLock);

        WaitForSingleObject(pEngineThreadContext->hWorkAvailable, INFINITE);
    }

    return 0;
}


/*******************************************************************************
 * Sudoku_FindBestSplitCell                                                                        
 *                                                                             
 * DESCRIPTION: 
 *                                                                             
 * INPUT                                                                       
 *   
 *                                                                             
 * OUTPUT                                                                       
 *   
 *                                                                             
 ******************************************************************************/
PSUDOKU_CELL Sudoku_FindBestSplitCell(PINTERNAL_SUDOKU pSudoku, PNODE_CELL_LOCATION pCellLocation)
{
    SUDOKU_FIND_BEST_CELL FindBestCell = {0};

    Sudoku_EnumNodesAndCellsByRows(pSudoku, Sudoku_FindBestSplitCell_Callback, &FindBestCell);

    *pCellLocation = FindBestCell.CellLocation;

    return FindBestCell.pBestCell;
}


/*******************************************************************************
 * Sudoku_FindBestSplitCell_Callback                                                                        
 *                                                                             
 * DESCRIPTION: 
 *                                                                             
 * INPUT                                                                       
 *   
 *                                                                             
 * OUTPUT                                                                       
 *   
 *                                                                             
 ******************************************************************************/
BOOL Sudoku_FindBestSplitCell_Callback(PNODE_CELL_ENUMERATION_CTX pNodeCellEnumeration)
{
    PSUDOKU_FIND_BEST_CELL pSudokuFindBestCell = (PSUDOKU_FIND_BEST_CELL)pNodeCellEnumeration->pContext;
    ULONG NumberOfCompletedCellsInCol;
    ULONG NumberOfCompletedCellsInNode;
    ULONG NumberOfCompletedCellsInRow;
    ULONG NumberOfPossibilities;
    ULONG TotalCompleted;
    ULONG MostCompleted;
    BOOL bNewBestCell;

    bNewBestCell = FALSE;

    if(pNodeCellEnumeration->pCell->Complete == FALSE)
    {
        NumberOfCompletedCellsInCol = Sudoku_NumerOfBits(*pNodeCellEnumeration->pCell->pCompletedColLookup);
        NumberOfCompletedCellsInNode = Sudoku_NumerOfBits(pNodeCellEnumeration->pNode->CompletedMask);
        NumberOfCompletedCellsInRow = Sudoku_NumerOfBits(*pNodeCellEnumeration->pCell->pCompletedRowLookup);
        NumberOfPossibilities = Sudoku_NumerOfBits(pNodeCellEnumeration->pCell->PossibleMask);
        TotalCompleted = NumberOfCompletedCellsInCol + NumberOfCompletedCellsInNode + NumberOfCompletedCellsInRow;

        MostCompleted = NumberOfCompletedCellsInCol;

        if(MostCompleted < NumberOfCompletedCellsInNode)
        {
            MostCompleted = NumberOfCompletedCellsInNode;
        }

        if(MostCompleted < NumberOfCompletedCellsInRow)
        {
            MostCompleted = NumberOfCompletedCellsInRow;
        }

        if(pSudokuFindBestCell->pBestCell == NULL)
        {
            bNewBestCell = TRUE;
        }
        else
        {
            if(NumberOfPossibilities < pSudokuFindBestCell->NumberOfPossibilities)
            {
                bNewBestCell = TRUE;
            }
            else
            {
                if(NumberOfPossibilities == pSudokuFindBestCell->NumberOfPossibilities)
                {
                    if(TotalCompleted > pSudokuFindBestCell->TotalCompleted)
                    {
                        bNewBestCell = TRUE;
                    }
                    else
                    {
                        if(pSudokuFindBestCell->TotalCompleted == TotalCompleted)
                        {
                            if(MostCompleted > pSudokuFindBestCell->MostCompleted)
                            {
                                bNewBestCell = TRUE;
                            }
                        }
                    }
                }
            }
        }

        if(bNewBestCell)
        {
            pSudokuFindBestCell->pBestCell = pNodeCellEnumeration->pCell;

            pSudokuFindBestCell->NumberOfCompletedCellsInCol = NumberOfCompletedCellsInCol;
            pSudokuFindBestCell->NumberOfCompletedCellsInNode = NumberOfCompletedCellsInNode;
            pSudokuFindBestCell->NumberOfCompletedCellsInRow = NumberOfCompletedCellsInRow;
            pSudokuFindBestCell->NumberOfPossibilities = NumberOfPossibilities;

            pSudokuFindBestCell->TotalCompleted = TotalCompleted;
            pSudokuFindBestCell->CellLocation = pNodeCellEnumeration->NodeCellLocation;
        }
    }

    return TRUE;
}



/*******************************************************************************
 * Sudoku_NextState                                                                        
 *                                                                             
 * DESCRIPTION: 
 *                                                                             
 * INPUT                                                                       
 *   
 *                                                                             
 * OUTPUT                                                                       
 *   
 *                                                                             
 ******************************************************************************/
void Sudoku_NextState(PINTERNAL_SUDOKU pSudoku)
{
    pSudoku->State = pSudoku->NextState;
    pSudoku->NextState = g_SudokuNextStateMapping[pSudoku->State];
    Sudoku_UpdateCounter(pSudoku, CounterTypeStateSwitch, 1);
}



/*******************************************************************************
 * Sudoku_SetNextState                                                                        
 *                                                                             
 * DESCRIPTION: 
 *                                                                             
 * INPUT                                                                       
 *   
 *                                                                             
 * OUTPUT                                                                       
 *   
 *                                                                             
 ******************************************************************************/
void Sudoku_SetNextState(PINTERNAL_SUDOKU pSudoku, SUDOKU_STATE SudokuState)
{
    if(pSudoku->NextState > SudokuState)
    {
        pSudoku->NextState = SudokuState;
    }
}


/*******************************************************************************
 * Sudoku_MaskCellPossibilities                                                                        
 *                                                                             
 * DESCRIPTION: 
 *                                                                             
 * INPUT                                                                       
 *   
 *                                                                             
 * OUTPUT                                                                       
 *   
 *                                                                             
 ******************************************************************************/
void Sudoku_MaskCellPossibilities(PINTERNAL_SUDOKU pSudoku, PSUDOKU_CELL pSudokuCell, ULONG Mask)
{
    pSudokuCell->PossibleMask &= Mask;
    Sudoku_SetNextState(pSudoku, SudokuStateVerifyConsistency);

    if(pSudokuCell->PossibleMask == 0)
    {
        Sudoku_SetNextState(pSudoku, SudokuStateFailure);
    }
}


/*******************************************************************************
 * Sudoku_ProcessBug                                                                        
 *                                                                             
 * DESCRIPTION: 
 *                                                                             
 * INPUT                                                                       
 *   
 *                                                                             
 * OUTPUT                                                                       
 *   
 *                                                                             
 ******************************************************************************/
void Sudoku_ProcessBug(PINTERNAL_SUDOKU pSudoku)
{
   Sudoku_UpdateCounter(pSudoku, CounterTypeBug, 1);
   Sudoku_StopEngine(pSudoku);
}


/*******************************************************************************
 * Sudoku_StopEngine                                                                        
 *                                     
 * DESCRIPTION:                        
 *                                     
 * INPUT                               
 *                                     
 *                                     
 * OUTPUT                              
 *                                     
 *                                     
 *******************************************************************************/

void Sudoku_StopEngine(PINTERNAL_SUDOKU pSudoku)
{
    pSudoku->StopEngine = TRUE;
}

/*******************************************************************************
 * Sudoku_StopAllEngines                                                                        
 *                                     
 * DESCRIPTION:                        
 *                                     
 * INPUT                               
 *                                     
 *                                     
 * OUTPUT                              
 *                                     
 *                                     
 *******************************************************************************/
void Sudoku_StopAllEngines(PINTERNAL_SUDOKU pSudoku)
{
    Sudoku_StopEngine(pSudoku);

    if(pSudoku->pSudokuGlobalSync)
    {
        if((pSudoku->Flags & SF_SOLVE_ALL_PATHS) == 0)
        {
            EnterCriticalSection(&pSudoku->pSudokuGlobalSync->csEngineLock);
    
            pSudoku->pSudokuGlobalSync->StopAllEngines = TRUE;
    
            LeaveCriticalSection(&pSudoku->pSudokuGlobalSync->csEngineLock);
        }
    }
}


/*******************************************************************************
 * Sudoku_UpdateCounter                                                                        
 *                                     
 * DESCRIPTION:                        
 *                                     
 * INPUT                               
 *                                     
 *                                     
 * OUTPUT                              
 *                                     
 *                                     
 *******************************************************************************/
void Sudoku_UpdateCounter(PINTERNAL_SUDOKU pSudoku, COUNTER_TYPE CounterType, ULONG Count)
{
    switch(CounterType)
    {
       case CounterTypeBug:
            pSudoku->EngineCounters.Bugs += Count;
            break;

       case CounterTypeFailure:
            pSudoku->EngineCounters.Failures += Count;
            break;

       case CounterTypeSplit:
            pSudoku->EngineCounters.Splits += Count;
            break;

       case CounterTypeDirections:
            pSudoku->EngineCounters.Directions += Count;
            break;

       case CounterTypeStateSwitch:
            pSudoku->EngineCounters.StateSwitches += Count;
            break;
    }
}

/*******************************************************************************
 * Sudoku_PreserveTotalEngineCounters                                                                        
 *                                     
 * DESCRIPTION:                        
 *                                     
 * INPUT                               
 *                                     
 *                                     
 * OUTPUT                              
 *                                     
 *                                     
 *******************************************************************************/
void Sudoku_PreserveTotalEngineCounters(PINTERNAL_SUDOKU pSudoku)
{
    pSudoku->pSudokuGlobalSync->TotalEngineCounters.Bugs        += pSudoku->EngineCounters.Bugs;
    pSudoku->pSudokuGlobalSync->TotalEngineCounters.Splits      += pSudoku->EngineCounters.Splits;
    pSudoku->pSudokuGlobalSync->TotalEngineCounters.Failures    += pSudoku->EngineCounters.Failures;
    pSudoku->pSudokuGlobalSync->TotalEngineCounters.Directions  += pSudoku->EngineCounters.Directions;
    pSudoku->pSudokuGlobalSync->TotalEngineCounters.StateSwitches  += pSudoku->EngineCounters.StateSwitches;
}



/*******************************************************************************
 * Sudoku_DisplayCounters                                                                        
 *                                     
 * DESCRIPTION:                        
 *                                     
 * INPUT                               
 *                                     
 *                                     
 * OUTPUT                              
 *                                     
 *                                     
 *******************************************************************************/
void Sudoku_DisplayCounters(PINTERNAL_SUDOKU pSudoku)
{
    UINT Index;

    if(pSudoku->pSudokuGlobalSync)
    {
        printf(" Bugs %i\n", pSudoku->EngineCounters.Bugs + pSudoku->pSudokuGlobalSync->TotalEngineCounters.Bugs);
        printf(" Failures %i \n", pSudoku->EngineCounters.Failures + pSudoku->pSudokuGlobalSync->TotalEngineCounters.Failures);
        printf(" Splits %i \n", pSudoku->EngineCounters.Splits + pSudoku->pSudokuGlobalSync->TotalEngineCounters.Splits);
        printf(" Directions %i \n", pSudoku->EngineCounters.Directions + pSudoku->pSudokuGlobalSync->TotalEngineCounters.Directions);
        printf(" State Switches %i \n", pSudoku->EngineCounters.StateSwitches + pSudoku->pSudokuGlobalSync->TotalEngineCounters.StateSwitches);

        printf(" Interrupts %i \n", pSudoku->pSudokuGlobalSync->ThreadCounters.Interrupts);

        for(Index = 0; Index < NUM_WORKER_QUEUES; Index++)
        {
            printf(" Work Items Scheduled Priority %i %i \n", Index, pSudoku->pSudokuGlobalSync->ThreadCounters.WorkItemsScheduled[Index]);
        }
    }
    else
    {
        printf(" Bugs %i\n", pSudoku->EngineCounters.Bugs);
        printf(" Failures %i \n", pSudoku->EngineCounters.Failures);
        printf(" Splits %i \n", pSudoku->EngineCounters.Splits);
        printf(" Directions %i \n", pSudoku->EngineCounters.Directions);
        printf(" State Switches %i \n", pSudoku->EngineCounters.StateSwitches);
    }
}


/*******************************************************************************
 * Sudoku_StartTimer                                                                        
 *                                     
 * DESCRIPTION:                        
 *                                     
 * INPUT                               
 *                                     
 *                                     
 * OUTPUT                              
 *                                     
 *                                     
 *******************************************************************************/
void Sudoku_StartTimer(PTIME_TRACKING pTimeTracking)
{
    QueryPerformanceCounter(&pTimeTracking->StartTime);    
}


/*******************************************************************************
 * Sudoku_Debug_Console_Display                                                                        
 *                                     
 * DESCRIPTION:                        
 *                                     
 * INPUT                               
 *                                     
 *                                     
 * OUTPUT                              
 *                                     
 *                                     
 *******************************************************************************/
void Sudoku_Debug_Console_Display(HSUDOKU hSudoku)
{
    PINTERNAL_SUDOKU pSudoku = (PINTERNAL_SUDOKU)hSudoku;
    PINTERNAL_SUDOKU pSudokuSuccessThread;

    if(pSudoku->SolveAttempted == FALSE)
    {
        printf("Initial Puzzle State\n");
        Sudoku_Debug_DisplayMatrix(pSudoku);
    }
    else
    {
        if(pSudoku->Solved)
        {
            printf(" Completed and Solved \n");
            if(pSudoku->pSudokuGlobalSync)
            {
                pSudokuSuccessThread = pSudoku->pSudokuGlobalSync->pSuccessConfiguration;

                while(pSudokuSuccessThread)
                {
                    printf("Solution \n");
                    Sudoku_Debug_DisplayMatrix(pSudokuSuccessThread);
                    pSudokuSuccessThread = pSudokuSuccessThread->pSolutionQueueNext;
                }
            }
            else
            {
                Sudoku_Debug_DisplayMatrix(pSudoku);
            }
        }
        else
        {
            printf(" Puzzle could not be solved\n");
            Sudoku_Debug_DisplayMatrix(pSudoku);
        }

        Sudoku_DisplayCounters(pSudoku);
        Sudoku_DisplayTime(&pSudoku->TimeTracking);
    }
}


/*******************************************************************************
 * Sudoku_StopTimer                                                                        
 *                                     
 * DESCRIPTION:                        
 *                                     
 * INPUT                               
 *                                     
 *                                     
 * OUTPUT                              
 *                                     
 *                                     
 *******************************************************************************/

void Sudoku_StopTimer(PTIME_TRACKING pTimeTracking)
{
    ULONG64 TimeDifference;

    QueryPerformanceCounter(&pTimeTracking->StopTime);    
    QueryPerformanceFrequency(&pTimeTracking->Frequency);

    TimeDifference = pTimeTracking->StopTime.QuadPart - pTimeTracking->StartTime.QuadPart;
    pTimeTracking->Seconds = (float)((double)TimeDifference/pTimeTracking->Frequency.QuadPart);
    pTimeTracking->Milliseconds = (float)((double)TimeDifference/((double)pTimeTracking->Frequency.QuadPart/1000.0));
    pTimeTracking->Microseconds = (float)((double)TimeDifference/((double)pTimeTracking->Frequency.QuadPart/1000000.0));
}

/*******************************************************************************
 * Sudoku_DisplayTime                                                                        
 *                                     
 * DESCRIPTION:                        
 *                                     
 * INPUT                               
 *                                     
 *                                     
 * OUTPUT                              
 *                                     
 *                                     
 *******************************************************************************/

void Sudoku_DisplayTime(PTIME_TRACKING pTimeTracking)
{
    printf(" Solution Time: (%1.3f Seconds) or (%1.3f Milliseconds) or (%1.3f Microseconds)\n", pTimeTracking->Seconds, pTimeTracking->Milliseconds, pTimeTracking->Microseconds);
}


/*******************************************************************************
 * Sudoku_DisplayTime                                                                        
 *                                     
 * DESCRIPTION:                        
 *                                     
 * INPUT                               
 *                                     
 *                                     
 * OUTPUT                              
 *                                     
 *                                     
 *******************************************************************************/

BOOL Sudoku_WorkerQueuesAreEmpty(PSUDOKU_GLOBAL_SYNC pSudokuEngine, ULONG LowestPriority)
{
    UINT Index;
    BOOL bWorkerQueueIsEmpty = TRUE;

    for(Index = LowestPriority; Index < NUM_WORKER_QUEUES && bWorkerQueueIsEmpty != FALSE; Index++)
    {
        if(pSudokuEngine->pWorkerQueue[Index])
        {
            bWorkerQueueIsEmpty = FALSE;
        }
    }

    return bWorkerQueueIsEmpty;
}

/*******************************************************************************
 * Sudoku_GetNextWorkItem                                                                        
 *                                     
 * DESCRIPTION:                        
 *                                     
 * INPUT                               
 *                                     
 *                                     
 * OUTPUT                              
 *                                     
 *                                     
 *******************************************************************************/

PINTERNAL_SUDOKU Sudoku_GetNextWorkItem(PSUDOKU_GLOBAL_SYNC pSudokuEngine, ULONG LowestPriority)
{
    PINTERNAL_SUDOKU pSudoku = NULL;

    UINT Index;
    UINT WorkerQueueIndex;
    BOOL bWorkerQueueIsEmpty = TRUE;

    WorkerQueueIndex = (NUM_WORKER_QUEUES-1);

    for(Index = LowestPriority; Index < NUM_WORKER_QUEUES && pSudoku == NULL; Index++, WorkerQueueIndex--)
    {
        if(pSudokuEngine->pWorkerQueue[WorkerQueueIndex])
        {
            pSudoku = pSudokuEngine->pWorkerQueue[WorkerQueueIndex];
            pSudokuEngine->pWorkerQueue[WorkerQueueIndex] = pSudoku->pWorkerQueueNext;
            pSudoku->pWorkerQueueNext = NULL;
            pSudoku->pSudokuGlobalSync->ThreadCounters.WorkItemsScheduled[WorkerQueueIndex]++;
        }
    }

    return pSudoku;
}



/*******************************************************************************
 * Sudoku_GetNextWorkItem                                                                        
 *                                     
 * DESCRIPTION:                        
 *                                     
 * INPUT                               
 *                                     
 *                                     
 * OUTPUT                              
 *                                     
 *                                     
 *******************************************************************************/

void Sudoku_PushWorkItem(PSUDOKU_GLOBAL_SYNC pSudokuEngine, PINTERNAL_SUDOKU pSudoku)
{
    pSudoku->pWorkerQueueNext = pSudokuEngine->pWorkerQueue[pSudoku->Priority];
    pSudokuEngine->pWorkerQueue[pSudoku->Priority] = pSudoku;
    SetEvent(pSudokuEngine->hWorkAvailable);
}


/*******************************************************************************
 * Sudoku_GetNextWorkItem                                                                        
 *                                     
 * DESCRIPTION:                        
 *                                     
 * INPUT                               
 *                                     
 *                                     
 * OUTPUT                              
 *                                     
 *                                     
 *******************************************************************************/

void Sudoku_UpdateContextPriority(PINTERNAL_SUDOKU pSudoku)
{
    switch(pSudoku->Flags & FS_PRIORITY_MASK)
    {
        case FS_PRIORITY_NUM_COMPLETED_ROWS: 
             pSudoku->Priority = pSudoku->NumberOfCompletedRows;
             break;

        case FS_PRIORITY_NUM_COMPLETED_COLS: 
             pSudoku->Priority = pSudoku->NumberOfCompletedCols;
             break;

        case FS_PRIORITY_NUM_COMPLETED_NODES:
        default:
             pSudoku->Priority = pSudoku->NumberOfCompletedNodes;
    }
}
 