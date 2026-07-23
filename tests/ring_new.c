/*
 *******************************************************************************************
 ** 黑洞检测法 —— 圆环识别核心函数组
 ** 基于 hao-yue-1/SmartCar (广东工业大学霹雳火队) 纯视觉方案
 ** 适配 TC264 + MT9V03X + 94x60压缩图
 ** 无IMU、无电磁 —— 纯二值图像素分析
 *******************************************************************************************
 */

/* ---- 黑洞底部检测：检查图像底部角落是否存在黑色区域 ---- */
static uint8 BlackHole_Check_Bottom(uint8 direction)
{
    int row, col;
    int black_count;
    
    if (direction == 1U) /* 左环岛：检查左下角 */
    {
        for (row = LCDH - 1; row >= BH_BOTTOM_START_ROW; row--)
        {
            black_count = 0;
            for (col = BH_LEFT_COL_MIN; col <= BH_LEFT_COL_MAX; col++)
            {
                if (Pixle[row][col] == IMG_BLACK) black_count++;
            }
            if (black_count >= (BH_LEFT_COL_MAX - BH_LEFT_COL_MIN + 1))
                return 1;
        }
    }
    else /* 右环岛：检查右下角 */
    {
        for (row = LCDH - 1; row >= BH_BOTTOM_START_ROW; row--)
        {
            black_count = 0;
            for (col = BH_RIGHT_COL_MIN; col <= BH_RIGHT_COL_MAX; col++)
            {
                if (Pixle[row][col] == IMG_BLACK) black_count++;
            }
            if (black_count >= (BH_RIGHT_COL_MAX - BH_RIGHT_COL_MIN + 1))
                return 1;
        }
    }
    return 0;
}

/* ---- 验证拐点上方是否存在黑洞 —— 区分普通弯道与圆环 ---- */
static uint8 BlackHole_Check_Above(int inflection_row, int inflection_col)
{
    int row;
    
    for (row = inflection_row - 2; row > BH_BOTTOM_START_ROW + 10; row--)
    {
        if (Pixle[row][inflection_col] == IMG_WHITE
            && Pixle[row + 1][inflection_col] == IMG_BLACK)
        {
            for (; row > BH_BOTTOM_START_ROW + 5; row--)
            {
                if (Pixle[row][inflection_col] == IMG_BLACK
                    && Pixle[row + 1][inflection_col] == IMG_WHITE)
                {
                    return 1;
                }
            }
            break;
        }
    }
    return 0;
}

/* ---- 谷底追踪：从白黑跳变点向右下/左下追踪到谷底 ---- */
static int BlackHole_Track_Valley(uint8 direction, int *valley_row, int *valley_col)
{
    int row, col;
    int moved;
    int scan_col;
    
    scan_col = (direction == 1U) ? VALLEY_SCAN_COL_LEFT : VALLEY_SCAN_COL_RIGHT;
    
    for (row = VALLEY_SCAN_START_ROW; row > VALLEY_MIN_ROW; row--)
    {
        if (Pixle[row][scan_col] == IMG_WHITE
            && Pixle[row - 1][scan_col] == IMG_BLACK)
        {
            col = scan_col;
            
            if (direction == 1U)
            {
                for (; col + 1 < LCDW - 1; col++)
                {
                    if (Pixle[row][col + 1] == IMG_WHITE) break;
                }
                do {
                    moved = 0;
                    if (col + 1 < LCDW - 1 && row + 1 < LCDH - 1
                        && Pixle[row][col + 1] == IMG_BLACK)
                    {
                        col++;
                        moved = 1;
                    }
                    if (row + 1 < LCDH - 1
                        && Pixle[row + 1][col] == IMG_BLACK)
                    {
                        row++;
                        moved = 1;
                    }
                } while (moved);
            }
            else
            {
                for (; col - 1 > 0; col--)
                {
                    if (Pixle[row][col - 1] == IMG_WHITE) break;
                }
                do {
                    moved = 0;
                    if (col - 1 > 0 && row + 1 < LCDH - 1
                        && Pixle[row][col - 1] == IMG_BLACK)
                    {
                        col--;
                        moved = 1;
                    }
                    if (row + 1 < LCDH - 1
                        && Pixle[row + 1][col] == IMG_BLACK)
                    {
                        row++;
                        moved = 1;
                    }
                } while (moved);
            }
            
            if (row > VALLEY_MIN_ROW && row < VALLEY_MAX_ROW
                && col > 0 && col < LCDW - 1)
            {
                *valley_row = row;
                *valley_col = col;
                return 1;
            }
            return 0;
        }
    }
    return 0;
}

/* ---- 候选检测：黑洞底部 + 丢线特征 ---- */
static uint8 Ring_Is_Candidate(uint8 direction)
{
    if (ImageStatus.OFFLine > 2)
        return 0U;
    if (!BlackHole_Check_Bottom(direction))
        return 0U;
    if (direction == 1U)
        return (uint8)(ImageStatus.Miss_Left_lines >= 10);
    if (direction == 2U)
        return (uint8)(ImageStatus.Miss_Right_lines >= 10);
    return 0U;
}

/* ---- 寻找入口谷底点：用谷底追踪替代边线跳变检测 ---- */
static int Ring_Find_Valley_Point(uint8 direction, int *valley_col)
{
    int valley_row = -1;
    int vcol = -1;
    
    if (BlackHole_Track_Valley(direction, &valley_row, &vcol) == 0)
    {
        *valley_col = -1;
        return -1;
    }
    *valley_col = vcol;
    return valley_row;
}

/* ---- 出环特征检测：出口侧边线恢复 + 黑洞验证 ---- */
static uint8 Ring_Has_Exit_Feature(uint8 direction)
{
    int row;
    
    if ((direction == 1U && ImageStatus.Miss_Right_lines > 4)
        || (direction == 2U && ImageStatus.Miss_Left_lines > 4))
        return 0U;
    
    for (row = SCAN_BASE_START_ROW - 1; row > 5; row--)
    {
        if (direction == 1U
            && ImageDeal[row].IsRightFind == 'T'
            && ImageDeal[row - 1].IsRightFind != 'T'
            && ImageDeal[row - 2].IsRightFind != 'T')
        {
            if (BlackHole_Check_Above(row, ImageDeal[row].RightBorder))
                return 1U;
        }
        if (direction == 2U
            && ImageDeal[row].IsLeftFind == 'T'
            && ImageDeal[row - 1].IsLeftFind != 'T'
            && ImageDeal[row - 2].IsLeftFind != 'T')
        {
            if (BlackHole_Check_Above(row, ImageDeal[row].LeftBorder))
                return 1U;
        }
    }
    return Ring_Is_Stable_Road();
}

/* ---- 补线偏移量 ---- */
static int Ring_Get_Fill_Offset(uint8 ring_state)
{
    switch (ring_state)
    {
    case RING_STATE_CONFIRM:
    case RING_STATE_APPROACH: return FILL_ENTRY_OFFSET;
    case RING_STATE_ENTRY:    return FILL_ENTRY_OFFSET;
    case RING_STATE_INSIDE:   return FILL_INSIDE_OFFSET;
    case RING_STATE_EXIT:     return FILL_EXIT_OFFSET;
    case RING_STATE_RECOVERY: return FILL_RECOVERY_OFFSET;
    default:                  return 0;
    }
}


/* ---- ????????????????????????? ---- */
/* ponytail: ??????????ImageDeal????????? */
static void Ring_Inside_Scan(uint8 direction)
{
    int row, col;
    int transition_cols[LCDH];       /* ?????????? */
    int scan_start;                  /* ?????? */
    int bottom_transition_col;       /* ?3???????? */
    int cut_off_row;                 /* ??????>???????Err?? */
    int fill_offset = FILL_INSIDE_OFFSET;
    
    /* ?????????Err?? */
    cut_off_row = SCAN_BASE_START_ROW + 1;
    
    /* ??????????????????????? */
    for (row = SCAN_BASE_START_ROW; row > ImageStatus.OFFLine; row--)
    {
        transition_cols[row] = -1;
        
        if (direction == 2U)  /* ??????????? */
        {
            /* ???? = ??????????????????? */
            scan_start = ImageDeal[row].RightBorder;
            if (scan_start <= 1 || scan_start >= LCDW - 1)
                scan_start = LCDW - 2;
            
            for (col = scan_start; col > 0; col--)
            {
                /* ??????: ??????????? */
                if (Pixle[row][col] != Pixle[row][col - 1])
                {
                    transition_cols[row] = col;
                    break;
                }
            }
            if (transition_cols[row] < 0)
                transition_cols[row] = 0;  /* ?????????0 */
        }
        else  /* ??????????? */
        {
            scan_start = ImageDeal[row].LeftBorder;
            if (scan_start <= 1 || scan_start >= LCDW - 1)
                scan_start = 2;
            
            for (col = scan_start; col < LCDW - 1; col++)
            {
                /* ??????: ??????????? */
                if (Pixle[row][col] != Pixle[row][col + 1])
                {
                    transition_cols[row] = col;
                    break;
                }
            }
            if (transition_cols[row] < 0)
                transition_cols[row] = LCDW - 1;  /* ??????????? */
        }
    }
    
    /* ?????????3?????????? */
    /* ponytail: SCAN_BASE_START_ROW=59??????????????? */
    {
        int row0 = SCAN_BASE_START_ROW;       /* ??? 59 */
        int row1 = SCAN_BASE_START_ROW - 1;   /* ??? 58 */
        int row2 = SCAN_BASE_START_ROW - 2;   /* ??? 57 */
        int bottom_same = 0;
        
        if (row2 > ImageStatus.OFFLine)
        {
            if (transition_cols[row0] >= 0
                && transition_cols[row0] == transition_cols[row1]
                && transition_cols[row1] == transition_cols[row2])
            {
                bottom_transition_col = transition_cols[row0];
                bottom_same = 1;
            }
            
            /* ??3???????????"???" */
            if (bottom_same)
            {
                if (direction == 2U)  /* ??????????????? */
                {
                    int edge_ref = ImageDeal[row0].RightBorder;
                    if (edge_ref <= 1 || edge_ref >= LCDW - 1)
                        edge_ref = LCDW - 1;
                    /* ???????3?????"????" */
                    if (bottom_transition_col < edge_ref - 3)
                        bottom_same = 0;  /* ????????? */
                }
                else  /* ??????????????? */
                {
                    int edge_ref = ImageDeal[row0].LeftBorder;
                    if (edge_ref <= 1 || edge_ref >= LCDW - 1)
                        edge_ref = 0;
                    if (bottom_transition_col > edge_ref + 3)
                        bottom_same = 0;
                }
            }
            
            if (bottom_same)
            {
                /* ?????3?????????????????????? */
                int found_row = -1;
                for (row = row2 - 1; row > ImageStatus.OFFLine; row--)
                {
                    if (transition_cols[row] >= 0
                        && transition_cols[row] != bottom_transition_col)
                    {
                        found_row = row;
                        break;
                    }
                }
                
                if (found_row >= 0)
                {
                    /* ?????found_row???: ??>found_row????Err?? */
                    cut_off_row = found_row;
                }
            }
        }
    }
    
    /* ??????????????? */
    for (row = SCAN_BASE_START_ROW; row > ImageStatus.OFFLine; row--)
    {
        if (row > cut_off_row)
        {
            /* ????????????????Err???? */
            /* ponytail: ??ImageSensorMid?????Err??????? */
            ImageDeal[row].Center = ImageSensorMid;
        }
        else if (transition_cols[row] >= 0)
        {
            if (direction == 2U)  /* ????????????(????) */
            {
                ImageDeal[row].Center = transition_cols[row] - fill_offset;
            }
            else  /* ????????????(????) */
            {
                ImageDeal[row].Center = transition_cols[row] + fill_offset;
            }
        }
        else
        {
            /* ???????????? */
            ImageDeal[row].Center = ImageSensorMid;
        }
        
        /* ????????????? */
        LimitL(ImageDeal[row].Center);
        LimitH(ImageDeal[row].Center);
    }
}

/* ---- 分段补线：入口/环内/出口/恢复 各阶段策略不同 ---- */
static void Ring_Rebuild_Fill(uint8 direction)
{
    int row;
    int valley_row, valley_col;
    uint8 ring_state = (uint8)ImageFlag.image_element_rings_flag;
    int fill_offset = Ring_Get_Fill_Offset(ring_state);
    int has_valley = BlackHole_Track_Valley(direction, &valley_row, &valley_col);
    
    switch (ring_state)
    {
    case RING_STATE_CONFIRM:
    case RING_STATE_APPROACH:
    case RING_STATE_ENTRY:
        for (row = SCAN_BASE_START_ROW; row > ImageStatus.OFFLine; row--)
        {
            if (direction == 1U)
                ImageDeal[row].Center = ImageDeal[row].RightBorder
                                      - Half_Bend_Wide[row] - fill_offset;
            else
                ImageDeal[row].Center = ImageDeal[row].LeftBorder
                                      + Half_Bend_Wide[row] + fill_offset;
            LimitL(ImageDeal[row].Center);
            LimitH(ImageDeal[row].Center);
        }
        break;
    case RING_STATE_INSIDE:
        /* ???????????????????????? */
        Ring_Inside_Scan(direction);
        break;
    case RING_STATE_EXIT:
        for (row = SCAN_BASE_START_ROW; row > ImageStatus.OFFLine; row--)
        {
            if (direction == 1U)
            {
                if (has_valley && row <= valley_row)
                    ImageDeal[row].Center = ImageDeal[row].RightBorder
                                          - Half_Bend_Wide[row] - fill_offset;
                else
                    ImageDeal[row].Center = ImageDeal[row].RightBorder
                                          - Half_Bend_Wide[row] - FILL_INSIDE_OFFSET;
            }
            else
            {
                if (has_valley && row <= valley_row)
                    ImageDeal[row].Center = ImageDeal[row].LeftBorder
                                          + Half_Bend_Wide[row] + fill_offset;
                else
                    ImageDeal[row].Center = ImageDeal[row].LeftBorder
                                          + Half_Bend_Wide[row] + FILL_INSIDE_OFFSET;
            }
            LimitL(ImageDeal[row].Center);
            LimitH(ImageDeal[row].Center);
        }
        break;
    case RING_STATE_RECOVERY:
    default:
        for (row = SCAN_BASE_START_ROW; row > ImageStatus.OFFLine; row--)
        {
            if (direction == 1U)
                ImageDeal[row].Center = (ImageDeal[row].RightBorder > 0)
                    ? ImageDeal[row].RightBorder - Half_Road_Wide[row] - fill_offset
                    : ImageSensorMid;
            else
                ImageDeal[row].Center = (ImageDeal[row].LeftBorder < LCDW - 1)
                    ? ImageDeal[row].LeftBorder + Half_Road_Wide[row] + fill_offset
                    : ImageSensorMid;
            LimitL(ImageDeal[row].Center);
            LimitH(ImageDeal[row].Center);
        }
        break;
    }
}

/* ---- 状态机更新 ---- */
static void Ring_State_Update(void)
{
    uint8 direction = (uint8)ImageFlag.image_element_rings;
    int valley_col = -1;
    int valley_row;

    if (direction != 1U && direction != 2U)
    {
        Ring_Clear_State();
        return;
    }
    if (s_ring_state_frames < 65535U)
        s_ring_state_frames++;

    switch (ImageFlag.image_element_rings_flag)
    {
    case RING_STATE_CONFIRM:
        if (Ring_Is_Candidate(direction))
        { if (s_ring_confirm_count < RING_CONFIRM_FRAMES) s_ring_confirm_count++; }
        else
        { s_ring_confirm_count = 0U; }
        if (s_ring_confirm_count >= RING_CONFIRM_FRAMES)
            Ring_Set_State(RING_STATE_APPROACH);
        else if (s_ring_state_frames >= RING_CONFIRM_MAX_FRAMES)
            Ring_Clear_State();
        break;

    case RING_STATE_APPROACH:
        valley_row = Ring_Find_Valley_Point(direction, &valley_col);
        if (valley_row >= 0)
        { s_ring_entry_corner_row = valley_row; s_ring_entry_corner_col = valley_col; }
        if (valley_row >= 0 && valley_row < VALLEY_MAX_ROW)
        { if (s_ring_feature_count < 3U) s_ring_feature_count++; }
        else
        { s_ring_feature_count = 0U; }
        if (s_ring_feature_count >= 3U || s_ring_state_frames >= RING_APPROACH_MAX_FRAMES)
            Ring_Set_State(RING_STATE_ENTRY);
        break;

    case RING_STATE_ENTRY:
        valley_row = Ring_Find_Valley_Point(direction, &valley_col);
        if (valley_row >= 0)
        { s_ring_entry_corner_row = valley_row; s_ring_entry_corner_col = valley_col; }
        if (valley_row < 0 || valley_row < VALLEY_MIN_ROW + 5)
        { if (s_ring_feature_count < 3U) s_ring_feature_count++; }
        else
        { s_ring_feature_count = 0U; }
        if (s_ring_feature_count >= 3U || s_ring_state_frames >= RING_ENTRY_MAX_FRAMES)
            Ring_Set_State(RING_STATE_INSIDE);
        break;

    case RING_STATE_INSIDE:
        if ((direction == 1U && ImageStatus.Miss_Right_lines >= EXIT_LOST_MIN)
            || (direction == 2U && ImageStatus.Miss_Left_lines >= EXIT_LOST_MIN)
            || ImageStatus.OFFLine >= EXIT_LOST_MIN)
            s_ring_exit_loss_seen = 1U;
        if (s_ring_exit_loss_seen && Ring_Has_Exit_Feature(direction))
        { if (s_ring_feature_count < RING_EXIT_CONFIRM_FRAMES) s_ring_feature_count++; }
        else
        { s_ring_feature_count = 0U; }
        if (s_ring_feature_count >= RING_EXIT_CONFIRM_FRAMES
            || s_ring_state_frames >= RING_INSIDE_MAX_FRAMES)
            Ring_Set_State(RING_STATE_EXIT);
        break;

    case RING_STATE_EXIT:
        if (Ring_Is_Stable_Road())
        { if (s_ring_stable_count < RING_EXIT_STABLE_FRAMES) s_ring_stable_count++; }
        else
        { s_ring_stable_count = 0U; }
        if (s_ring_stable_count >= RING_EXIT_STABLE_FRAMES
            || s_ring_state_frames >= RING_EXIT_MAX_FRAMES)
            Ring_Set_State(RING_STATE_RECOVERY);
        break;

    case RING_STATE_RECOVERY:
        if (ImageStatus.Miss_Left_lines < 4 && ImageStatus.Miss_Right_lines < 4
            && ImageStatus.OFFLine <= 2)
        { if (s_ring_stable_count < 4U) s_ring_stable_count++; }
        else
        { s_ring_stable_count = 0U; }
        if ((s_ring_state_frames >= RING_RECOVERY_FRAMES && s_ring_stable_count >= 4U)
            || s_ring_state_frames >= RING_RECOVERY_MAX_FRAMES)
            Ring_Clear_State();
        break;

    default:
        Ring_Clear_State();
        break;
    }
}

/* ---- 左圆环判断：黑洞检测触发 ---- */
void Element_Judgment_Left_Rings(void)
{
    if (ImageStatus.Miss_Right_lines > 5
        || ImageStatus.Miss_Left_lines < 10
        || ImageStatus.OFFLine > 2
        || ImageFlag.image_element_rings || ImageFlag.Out_Road == 1)
        return;

    { int r; for (r = SCAN_BASE_START_ROW; r >= SCAN_BASE_END_ROW; r--)
    { if (ImageDeal[r].IsLeftFind == 'W') return; } }

    if (BlackHole_Check_Bottom(1U))
    { ImageFlag.image_element_rings = 1; Ring_Set_State(RING_STATE_CONFIRM); }
}

/* ---- 右圆环判断：黑洞检测触发 ---- */
void Element_Judgment_Right_Rings(void)
{
    if (ImageStatus.Miss_Left_lines > 5
        || ImageStatus.Miss_Right_lines < 10
        || ImageStatus.OFFLine > 2
        || ImageFlag.image_element_rings || ImageFlag.Out_Road == 1)
        return;

    { int r; for (r = SCAN_BASE_START_ROW; r >= SCAN_BASE_END_ROW; r--)
    { if (ImageDeal[r].IsRightFind == 'W') return; } }

    if (BlackHole_Check_Bottom(2U))
    { ImageFlag.image_element_rings = 2; Ring_Set_State(RING_STATE_CONFIRM); }
}
