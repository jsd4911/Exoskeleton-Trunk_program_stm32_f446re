/**
  ******************************************************************************
  * @file    network.c
  * @author  AST Embedded Analytics Research Platform
  * @date    2026-05-15T21:31:31+0800
  * @brief   AI Tool Automatic Code Generator for Embedded NN computing
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  ******************************************************************************
  */


#include "network.h"
#include "network_data.h"

#include "ai_platform.h"
#include "ai_platform_interface.h"
#include "ai_math_helpers.h"

#include "core_common.h"
#include "core_convert.h"

#include "layers.h"



#undef AI_NET_OBJ_INSTANCE
#define AI_NET_OBJ_INSTANCE g_network
 
#undef AI_NETWORK_MODEL_SIGNATURE
#define AI_NETWORK_MODEL_SIGNATURE     "0x30622cebbf468f7724a7f660b889bf1e"

#ifndef AI_TOOLS_REVISION_ID
#define AI_TOOLS_REVISION_ID     ""
#endif

#undef AI_TOOLS_DATE_TIME
#define AI_TOOLS_DATE_TIME   "2026-05-15T21:31:31+0800"

#undef AI_TOOLS_COMPILE_TIME
#define AI_TOOLS_COMPILE_TIME    __DATE__ " " __TIME__

#undef AI_NETWORK_N_BATCHES
#define AI_NETWORK_N_BATCHES         (1)

static ai_ptr g_network_activations_map[1] = AI_C_ARRAY_INIT;
static ai_ptr g_network_weights_map[1] = AI_C_ARRAY_INIT;



/**  Array declarations section  **********************************************/
/* Array#0 */
AI_ARRAY_OBJ_DECLARE(
  input_6_features_output_array, AI_ARRAY_FORMAT_FLOAT|AI_FMT_FLAG_IS_IO,
  NULL, NULL, 60, AI_STATIC)

/* Array#1 */
AI_ARRAY_OBJ_DECLARE(
  input_6_features_Transpose_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 60, AI_STATIC)

/* Array#2 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 80, AI_STATIC)

/* Array#3 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_1_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 160, AI_STATIC)

/* Array#4 */
AI_ARRAY_OBJ_DECLARE(
  relu_1_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 160, AI_STATIC)

/* Array#5 */
AI_ARRAY_OBJ_DECLARE(
  linear_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 64, AI_STATIC)

/* Array#6 */
AI_ARRAY_OBJ_DECLARE(
  relu_2_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 64, AI_STATIC)

/* Array#7 */
AI_ARRAY_OBJ_DECLARE(
  out_move_output_array, AI_ARRAY_FORMAT_FLOAT|AI_FMT_FLAG_IS_IO,
  NULL, NULL, 2, AI_STATIC)

/* Array#8 */
AI_ARRAY_OBJ_DECLARE(
  out_bend_output_array, AI_ARRAY_FORMAT_FLOAT|AI_FMT_FLAG_IS_IO,
  NULL, NULL, 7, AI_STATIC)

/* Array#9 */
AI_ARRAY_OBJ_DECLARE(
  out_speed_output_array, AI_ARRAY_FORMAT_FLOAT|AI_FMT_FLAG_IS_IO,
  NULL, NULL, 3, AI_STATIC)

/* Array#10 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 288, AI_STATIC)

/* Array#11 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 16, AI_STATIC)

/* Array#12 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_1_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1536, AI_STATIC)

/* Array#13 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_1_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 32, AI_STATIC)

/* Array#14 */
AI_ARRAY_OBJ_DECLARE(
  linear_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 10240, AI_STATIC)

/* Array#15 */
AI_ARRAY_OBJ_DECLARE(
  linear_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 64, AI_STATIC)

/* Array#16 */
AI_ARRAY_OBJ_DECLARE(
  out_move_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 128, AI_STATIC)

/* Array#17 */
AI_ARRAY_OBJ_DECLARE(
  out_move_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 2, AI_STATIC)

/* Array#18 */
AI_ARRAY_OBJ_DECLARE(
  out_bend_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 448, AI_STATIC)

/* Array#19 */
AI_ARRAY_OBJ_DECLARE(
  out_bend_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 7, AI_STATIC)

/* Array#20 */
AI_ARRAY_OBJ_DECLARE(
  out_speed_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 192, AI_STATIC)

/* Array#21 */
AI_ARRAY_OBJ_DECLARE(
  out_speed_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 3, AI_STATIC)

/* Array#22 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_scratch0_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 18, AI_STATIC)

/* Array#23 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_scratch1_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 32, AI_STATIC)

/* Array#24 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_1_scratch0_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 48, AI_STATIC)

/**  Tensor declarations section  *********************************************/
/* Tensor #0 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_1_bias, AI_STATIC,
  0, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 1, 1), AI_STRIDE_INIT(4, 4, 4, 128, 128),
  1, &conv1d_1_bias_array, NULL)

/* Tensor #1 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_1_output, AI_STATIC,
  1, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 1, 5), AI_STRIDE_INIT(4, 4, 4, 128, 128),
  1, &conv1d_1_output_array, NULL)

/* Tensor #2 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_1_scratch0, AI_STATIC,
  2, 0x0,
  AI_SHAPE_INIT(4, 1, 16, 1, 3), AI_STRIDE_INIT(4, 4, 4, 64, 64),
  1, &conv1d_1_scratch0_array, NULL)

/* Tensor #3 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_1_weights, AI_STATIC,
  3, 0x0,
  AI_SHAPE_INIT(4, 16, 1, 3, 32), AI_STRIDE_INIT(4, 4, 64, 2048, 2048),
  1, &conv1d_1_weights_array, NULL)

/* Tensor #4 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_bias, AI_STATIC,
  4, 0x0,
  AI_SHAPE_INIT(4, 1, 16, 1, 1), AI_STRIDE_INIT(4, 4, 4, 64, 64),
  1, &conv1d_bias_array, NULL)

/* Tensor #5 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_output, AI_STATIC,
  5, 0x0,
  AI_SHAPE_INIT(4, 1, 16, 1, 5), AI_STRIDE_INIT(4, 4, 4, 64, 64),
  1, &conv1d_output_array, NULL)

/* Tensor #6 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_scratch0, AI_STATIC,
  6, 0x0,
  AI_SHAPE_INIT(4, 1, 6, 1, 3), AI_STRIDE_INIT(4, 4, 4, 24, 24),
  1, &conv1d_scratch0_array, NULL)

/* Tensor #7 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_scratch1, AI_STATIC,
  7, 0x0,
  AI_SHAPE_INIT(4, 1, 16, 1, 2), AI_STRIDE_INIT(4, 4, 4, 64, 64),
  1, &conv1d_scratch1_array, NULL)

/* Tensor #8 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_weights, AI_STATIC,
  8, 0x0,
  AI_SHAPE_INIT(4, 6, 1, 3, 16), AI_STRIDE_INIT(4, 4, 24, 384, 384),
  1, &conv1d_weights_array, NULL)

/* Tensor #9 */
AI_TENSOR_OBJ_DECLARE(
  input_6_features_Transpose_output, AI_STATIC,
  9, 0x0,
  AI_SHAPE_INIT(4, 1, 6, 1, 10), AI_STRIDE_INIT(4, 4, 4, 24, 24),
  1, &input_6_features_Transpose_output_array, NULL)

/* Tensor #10 */
AI_TENSOR_OBJ_DECLARE(
  input_6_features_output, AI_STATIC,
  10, 0x0,
  AI_SHAPE_INIT(4, 1, 10, 1, 6), AI_STRIDE_INIT(4, 4, 4, 40, 40),
  1, &input_6_features_output_array, NULL)

/* Tensor #11 */
AI_TENSOR_OBJ_DECLARE(
  linear_bias, AI_STATIC,
  11, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 1), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &linear_bias_array, NULL)

/* Tensor #12 */
AI_TENSOR_OBJ_DECLARE(
  linear_output, AI_STATIC,
  12, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 1), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &linear_output_array, NULL)

/* Tensor #13 */
AI_TENSOR_OBJ_DECLARE(
  linear_weights, AI_STATIC,
  13, 0x0,
  AI_SHAPE_INIT(4, 160, 64, 1, 1), AI_STRIDE_INIT(4, 4, 640, 40960, 40960),
  1, &linear_weights_array, NULL)

/* Tensor #14 */
AI_TENSOR_OBJ_DECLARE(
  out_bend_bias, AI_STATIC,
  14, 0x0,
  AI_SHAPE_INIT(4, 1, 7, 1, 1), AI_STRIDE_INIT(4, 4, 4, 28, 28),
  1, &out_bend_bias_array, NULL)

/* Tensor #15 */
AI_TENSOR_OBJ_DECLARE(
  out_bend_output, AI_STATIC,
  15, 0x0,
  AI_SHAPE_INIT(4, 1, 7, 1, 1), AI_STRIDE_INIT(4, 4, 4, 28, 28),
  1, &out_bend_output_array, NULL)

/* Tensor #16 */
AI_TENSOR_OBJ_DECLARE(
  out_bend_weights, AI_STATIC,
  16, 0x0,
  AI_SHAPE_INIT(4, 64, 7, 1, 1), AI_STRIDE_INIT(4, 4, 256, 1792, 1792),
  1, &out_bend_weights_array, NULL)

/* Tensor #17 */
AI_TENSOR_OBJ_DECLARE(
  out_move_bias, AI_STATIC,
  17, 0x0,
  AI_SHAPE_INIT(4, 1, 2, 1, 1), AI_STRIDE_INIT(4, 4, 4, 8, 8),
  1, &out_move_bias_array, NULL)

/* Tensor #18 */
AI_TENSOR_OBJ_DECLARE(
  out_move_output, AI_STATIC,
  18, 0x0,
  AI_SHAPE_INIT(4, 1, 2, 1, 1), AI_STRIDE_INIT(4, 4, 4, 8, 8),
  1, &out_move_output_array, NULL)

/* Tensor #19 */
AI_TENSOR_OBJ_DECLARE(
  out_move_weights, AI_STATIC,
  19, 0x0,
  AI_SHAPE_INIT(4, 64, 2, 1, 1), AI_STRIDE_INIT(4, 4, 256, 512, 512),
  1, &out_move_weights_array, NULL)

/* Tensor #20 */
AI_TENSOR_OBJ_DECLARE(
  out_speed_bias, AI_STATIC,
  20, 0x0,
  AI_SHAPE_INIT(4, 1, 3, 1, 1), AI_STRIDE_INIT(4, 4, 4, 12, 12),
  1, &out_speed_bias_array, NULL)

/* Tensor #21 */
AI_TENSOR_OBJ_DECLARE(
  out_speed_output, AI_STATIC,
  21, 0x0,
  AI_SHAPE_INIT(4, 1, 3, 1, 1), AI_STRIDE_INIT(4, 4, 4, 12, 12),
  1, &out_speed_output_array, NULL)

/* Tensor #22 */
AI_TENSOR_OBJ_DECLARE(
  out_speed_weights, AI_STATIC,
  22, 0x0,
  AI_SHAPE_INIT(4, 64, 3, 1, 1), AI_STRIDE_INIT(4, 4, 256, 768, 768),
  1, &out_speed_weights_array, NULL)

/* Tensor #23 */
AI_TENSOR_OBJ_DECLARE(
  relu_1_output, AI_STATIC,
  23, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 1, 5), AI_STRIDE_INIT(4, 4, 4, 128, 128),
  1, &relu_1_output_array, NULL)

/* Tensor #24 */
AI_TENSOR_OBJ_DECLARE(
  relu_1_output0, AI_STATIC,
  24, 0x0,
  AI_SHAPE_INIT(4, 1, 160, 1, 1), AI_STRIDE_INIT(4, 4, 4, 640, 640),
  1, &relu_1_output_array, NULL)

/* Tensor #25 */
AI_TENSOR_OBJ_DECLARE(
  relu_2_output, AI_STATIC,
  25, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 1), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &relu_2_output_array, NULL)



/**  Layer declarations section  **********************************************/


AI_TENSOR_CHAIN_OBJ_DECLARE(
  out_speed_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &relu_2_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &out_speed_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &out_speed_weights, &out_speed_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  out_speed_layer, 11,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense,
  &out_speed_chain,
  NULL, &out_speed_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  out_bend_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &relu_2_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &out_bend_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &out_bend_weights, &out_bend_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  out_bend_layer, 10,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense,
  &out_bend_chain,
  NULL, &out_speed_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  out_move_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &relu_2_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &out_move_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &out_move_weights, &out_move_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  out_move_layer, 9,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense,
  &out_move_chain,
  NULL, &out_bend_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  relu_2_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &linear_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &relu_2_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  relu_2_layer, 8,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &relu_2_chain,
  NULL, &out_move_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  linear_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &relu_1_output0),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &linear_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &linear_weights, &linear_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  linear_layer, 7,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense,
  &linear_chain,
  NULL, &relu_2_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  relu_1_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv1d_1_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &relu_1_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  relu_1_layer, 5,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &relu_1_chain,
  NULL, &linear_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  conv1d_1_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv1d_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv1d_1_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &conv1d_1_weights, &conv1d_1_bias, NULL),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &conv1d_1_scratch0, NULL)
)

AI_LAYER_OBJ_DECLARE(
  conv1d_1_layer, 4,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32,
  &conv1d_1_chain,
  NULL, &relu_1_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 1, 0, 1, 0), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  conv1d_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &input_6_features_Transpose_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv1d_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &conv1d_weights, &conv1d_bias, NULL),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &conv1d_scratch0, &conv1d_scratch1)
)

AI_LAYER_OBJ_DECLARE(
  conv1d_layer, 3,
  OPTIMIZED_CONV2D_TYPE, 0x0, NULL,
  conv2d_nl_pool, forward_conv2d_if32of32wf32_nl_pool,
  &conv1d_chain,
  NULL, &conv1d_1_layer, AI_STATIC, 
  .groups = 1, 
  .nl_params = NULL, 
  .nl_func = AI_HANDLE_PTR(forward_lite_nl_relu_if32of32), 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 1, 0, 1, 0), 
  .pool_size = AI_SHAPE_2D_INIT(1, 2), 
  .pool_stride = AI_SHAPE_2D_INIT(1, 2), 
  .pool_pad = AI_SHAPE_INIT(4, 0, 0, 0, 0), 
  .pool_func = AI_HANDLE_PTR(pool_func_mp_array_f32), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  input_6_features_Transpose_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &input_6_features_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &input_6_features_Transpose_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  input_6_features_Transpose_layer, 2,
  TRANSPOSE_TYPE, 0x0, NULL,
  transpose, forward_transpose,
  &input_6_features_Transpose_chain,
  NULL, &conv1d_layer, AI_STATIC, 
  .out_mapping = AI_SHAPE_INIT(6, AI_SHAPE_IN_CHANNEL, AI_SHAPE_HEIGHT, AI_SHAPE_WIDTH, AI_SHAPE_CHANNEL, AI_SHAPE_DEPTH, AI_SHAPE_EXTENSION), 
)


#if (AI_TOOLS_API_VERSION < AI_TOOLS_API_VERSION_1_5)

AI_NETWORK_OBJ_DECLARE(
  AI_NET_OBJ_INSTANCE, AI_STATIC,
  AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
    AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 51824, 1, 1),
    51824, NULL, NULL),
  AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
    AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 1152, 1, 1),
    1152, NULL, NULL),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_NETWORK_IN_NUM, &input_6_features_output),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_NETWORK_OUT_NUM, &out_move_output, &out_bend_output, &out_speed_output),
  &input_6_features_Transpose_layer, 0x28d89367, NULL)

#else

AI_NETWORK_OBJ_DECLARE(
  AI_NET_OBJ_INSTANCE, AI_STATIC,
  AI_BUFFER_ARRAY_OBJ_INIT_STATIC(
  	AI_FLAG_NONE, 1,
    AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
      AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 51824, 1, 1),
      51824, NULL, NULL)
  ),
  AI_BUFFER_ARRAY_OBJ_INIT_STATIC(
  	AI_FLAG_NONE, 1,
    AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
      AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 1152, 1, 1),
      1152, NULL, NULL)
  ),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_NETWORK_IN_NUM, &input_6_features_output),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_NETWORK_OUT_NUM, &out_move_output, &out_bend_output, &out_speed_output),
  &input_6_features_Transpose_layer, 0x28d89367, NULL)

#endif	/*(AI_TOOLS_API_VERSION < AI_TOOLS_API_VERSION_1_5)*/



/******************************************************************************/
AI_DECLARE_STATIC
ai_bool network_configure_activations(
  ai_network* net_ctx, const ai_network_params* params)
{
  AI_ASSERT(net_ctx)

  if (ai_platform_get_activations_map(g_network_activations_map, 1, params)) {
    /* Updating activations (byte) offsets */
    
    input_6_features_output_array.data = AI_PTR(g_network_activations_map[0] + 352);
    input_6_features_output_array.data_start = AI_PTR(g_network_activations_map[0] + 352);
    input_6_features_Transpose_output_array.data = AI_PTR(g_network_activations_map[0] + 592);
    input_6_features_Transpose_output_array.data_start = AI_PTR(g_network_activations_map[0] + 592);
    conv1d_scratch0_array.data = AI_PTR(g_network_activations_map[0] + 352);
    conv1d_scratch0_array.data_start = AI_PTR(g_network_activations_map[0] + 352);
    conv1d_scratch1_array.data = AI_PTR(g_network_activations_map[0] + 424);
    conv1d_scratch1_array.data_start = AI_PTR(g_network_activations_map[0] + 424);
    conv1d_output_array.data = AI_PTR(g_network_activations_map[0] + 832);
    conv1d_output_array.data_start = AI_PTR(g_network_activations_map[0] + 832);
    conv1d_1_scratch0_array.data = AI_PTR(g_network_activations_map[0] + 640);
    conv1d_1_scratch0_array.data_start = AI_PTR(g_network_activations_map[0] + 640);
    conv1d_1_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    conv1d_1_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    relu_1_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    relu_1_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    linear_output_array.data = AI_PTR(g_network_activations_map[0] + 640);
    linear_output_array.data_start = AI_PTR(g_network_activations_map[0] + 640);
    relu_2_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    relu_2_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    out_move_output_array.data = AI_PTR(g_network_activations_map[0] + 256);
    out_move_output_array.data_start = AI_PTR(g_network_activations_map[0] + 256);
    out_bend_output_array.data = AI_PTR(g_network_activations_map[0] + 264);
    out_bend_output_array.data_start = AI_PTR(g_network_activations_map[0] + 264);
    out_speed_output_array.data = AI_PTR(g_network_activations_map[0] + 292);
    out_speed_output_array.data_start = AI_PTR(g_network_activations_map[0] + 292);
    return true;
  }
  AI_ERROR_TRAP(net_ctx, INIT_FAILED, NETWORK_ACTIVATIONS);
  return false;
}




/******************************************************************************/
AI_DECLARE_STATIC
ai_bool network_configure_weights(
  ai_network* net_ctx, const ai_network_params* params)
{
  AI_ASSERT(net_ctx)

  if (ai_platform_get_weights_map(g_network_weights_map, 1, params)) {
    /* Updating weights (byte) offsets */
    
    conv1d_weights_array.format |= AI_FMT_FLAG_CONST;
    conv1d_weights_array.data = AI_PTR(g_network_weights_map[0] + 0);
    conv1d_weights_array.data_start = AI_PTR(g_network_weights_map[0] + 0);
    conv1d_bias_array.format |= AI_FMT_FLAG_CONST;
    conv1d_bias_array.data = AI_PTR(g_network_weights_map[0] + 1152);
    conv1d_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 1152);
    conv1d_1_weights_array.format |= AI_FMT_FLAG_CONST;
    conv1d_1_weights_array.data = AI_PTR(g_network_weights_map[0] + 1216);
    conv1d_1_weights_array.data_start = AI_PTR(g_network_weights_map[0] + 1216);
    conv1d_1_bias_array.format |= AI_FMT_FLAG_CONST;
    conv1d_1_bias_array.data = AI_PTR(g_network_weights_map[0] + 7360);
    conv1d_1_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 7360);
    linear_weights_array.format |= AI_FMT_FLAG_CONST;
    linear_weights_array.data = AI_PTR(g_network_weights_map[0] + 7488);
    linear_weights_array.data_start = AI_PTR(g_network_weights_map[0] + 7488);
    linear_bias_array.format |= AI_FMT_FLAG_CONST;
    linear_bias_array.data = AI_PTR(g_network_weights_map[0] + 48448);
    linear_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 48448);
    out_move_weights_array.format |= AI_FMT_FLAG_CONST;
    out_move_weights_array.data = AI_PTR(g_network_weights_map[0] + 48704);
    out_move_weights_array.data_start = AI_PTR(g_network_weights_map[0] + 48704);
    out_move_bias_array.format |= AI_FMT_FLAG_CONST;
    out_move_bias_array.data = AI_PTR(g_network_weights_map[0] + 49216);
    out_move_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 49216);
    out_bend_weights_array.format |= AI_FMT_FLAG_CONST;
    out_bend_weights_array.data = AI_PTR(g_network_weights_map[0] + 49224);
    out_bend_weights_array.data_start = AI_PTR(g_network_weights_map[0] + 49224);
    out_bend_bias_array.format |= AI_FMT_FLAG_CONST;
    out_bend_bias_array.data = AI_PTR(g_network_weights_map[0] + 51016);
    out_bend_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 51016);
    out_speed_weights_array.format |= AI_FMT_FLAG_CONST;
    out_speed_weights_array.data = AI_PTR(g_network_weights_map[0] + 51044);
    out_speed_weights_array.data_start = AI_PTR(g_network_weights_map[0] + 51044);
    out_speed_bias_array.format |= AI_FMT_FLAG_CONST;
    out_speed_bias_array.data = AI_PTR(g_network_weights_map[0] + 51812);
    out_speed_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 51812);
    return true;
  }
  AI_ERROR_TRAP(net_ctx, INIT_FAILED, NETWORK_WEIGHTS);
  return false;
}


/**  PUBLIC APIs SECTION  *****************************************************/



AI_DEPRECATED
AI_API_ENTRY
ai_bool ai_network_get_info(
  ai_handle network, ai_network_report* report)
{
  ai_network* net_ctx = AI_NETWORK_ACQUIRE_CTX(network);

  if (report && net_ctx)
  {
    ai_network_report r = {
      .model_name        = AI_NETWORK_MODEL_NAME,
      .model_signature   = AI_NETWORK_MODEL_SIGNATURE,
      .model_datetime    = AI_TOOLS_DATE_TIME,
      
      .compile_datetime  = AI_TOOLS_COMPILE_TIME,
      
      .runtime_revision  = ai_platform_runtime_get_revision(),
      .runtime_version   = ai_platform_runtime_get_version(),

      .tool_revision     = AI_TOOLS_REVISION_ID,
      .tool_version      = {AI_TOOLS_VERSION_MAJOR, AI_TOOLS_VERSION_MINOR,
                            AI_TOOLS_VERSION_MICRO, 0x0},
      .tool_api_version  = AI_STRUCT_INIT,

      .api_version            = ai_platform_api_get_version(),
      .interface_api_version  = ai_platform_interface_api_get_version(),
      
      .n_macc            = 22266,
      .n_inputs          = 0,
      .inputs            = NULL,
      .n_outputs         = 0,
      .outputs           = NULL,
      .params            = AI_STRUCT_INIT,
      .activations       = AI_STRUCT_INIT,
      .n_nodes           = 0,
      .signature         = 0x28d89367,
    };

    if (!ai_platform_api_get_network_report(network, &r)) return false;

    *report = r;
    return true;
  }
  return false;
}



AI_API_ENTRY
ai_bool ai_network_get_report(
  ai_handle network, ai_network_report* report)
{
  ai_network* net_ctx = AI_NETWORK_ACQUIRE_CTX(network);

  if (report && net_ctx)
  {
    ai_network_report r = {
      .model_name        = AI_NETWORK_MODEL_NAME,
      .model_signature   = AI_NETWORK_MODEL_SIGNATURE,
      .model_datetime    = AI_TOOLS_DATE_TIME,
      
      .compile_datetime  = AI_TOOLS_COMPILE_TIME,
      
      .runtime_revision  = ai_platform_runtime_get_revision(),
      .runtime_version   = ai_platform_runtime_get_version(),

      .tool_revision     = AI_TOOLS_REVISION_ID,
      .tool_version      = {AI_TOOLS_VERSION_MAJOR, AI_TOOLS_VERSION_MINOR,
                            AI_TOOLS_VERSION_MICRO, 0x0},
      .tool_api_version  = AI_STRUCT_INIT,

      .api_version            = ai_platform_api_get_version(),
      .interface_api_version  = ai_platform_interface_api_get_version(),
      
      .n_macc            = 22266,
      .n_inputs          = 0,
      .inputs            = NULL,
      .n_outputs         = 0,
      .outputs           = NULL,
      .map_signature     = AI_MAGIC_SIGNATURE,
      .map_weights       = AI_STRUCT_INIT,
      .map_activations   = AI_STRUCT_INIT,
      .n_nodes           = 0,
      .signature         = 0x28d89367,
    };

    if (!ai_platform_api_get_network_report(network, &r)) return false;

    *report = r;
    return true;
  }
  return false;
}


AI_API_ENTRY
ai_error ai_network_get_error(ai_handle network)
{
  return ai_platform_network_get_error(network);
}


AI_API_ENTRY
ai_error ai_network_create(
  ai_handle* network, const ai_buffer* network_config)
{
  return ai_platform_network_create(
    network, network_config, 
    AI_CONTEXT_OBJ(&AI_NET_OBJ_INSTANCE),
    AI_TOOLS_API_VERSION_MAJOR, AI_TOOLS_API_VERSION_MINOR, AI_TOOLS_API_VERSION_MICRO);
}


AI_API_ENTRY
ai_error ai_network_create_and_init(
  ai_handle* network, const ai_handle activations[], const ai_handle weights[])
{
  ai_error err;
  ai_network_params params;

  err = ai_network_create(network, AI_NETWORK_DATA_CONFIG);
  if (err.type != AI_ERROR_NONE) {
    return err;
  }
  
  if (ai_network_data_params_get(&params) != true) {
    err = ai_network_get_error(*network);
    return err;
  }
#if defined(AI_NETWORK_DATA_ACTIVATIONS_COUNT)
  /* set the addresses of the activations buffers */
  for (ai_u16 idx=0; activations && idx<params.map_activations.size; idx++) {
    AI_BUFFER_ARRAY_ITEM_SET_ADDRESS(&params.map_activations, idx, activations[idx]);
  }
#endif
#if defined(AI_NETWORK_DATA_WEIGHTS_COUNT)
  /* set the addresses of the weight buffers */
  for (ai_u16 idx=0; weights && idx<params.map_weights.size; idx++) {
    AI_BUFFER_ARRAY_ITEM_SET_ADDRESS(&params.map_weights, idx, weights[idx]);
  }
#endif
  if (ai_network_init(*network, &params) != true) {
    err = ai_network_get_error(*network);
  }
  return err;
}


AI_API_ENTRY
ai_buffer* ai_network_inputs_get(ai_handle network, ai_u16 *n_buffer)
{
  if (network == AI_HANDLE_NULL) {
    network = (ai_handle)&AI_NET_OBJ_INSTANCE;
    AI_NETWORK_OBJ(network)->magic = AI_MAGIC_CONTEXT_TOKEN;
  }
  return ai_platform_inputs_get(network, n_buffer);
}


AI_API_ENTRY
ai_buffer* ai_network_outputs_get(ai_handle network, ai_u16 *n_buffer)
{
  if (network == AI_HANDLE_NULL) {
    network = (ai_handle)&AI_NET_OBJ_INSTANCE;
    AI_NETWORK_OBJ(network)->magic = AI_MAGIC_CONTEXT_TOKEN;
  }
  return ai_platform_outputs_get(network, n_buffer);
}


AI_API_ENTRY
ai_handle ai_network_destroy(ai_handle network)
{
  return ai_platform_network_destroy(network);
}


AI_API_ENTRY
ai_bool ai_network_init(
  ai_handle network, const ai_network_params* params)
{
  ai_network* net_ctx = AI_NETWORK_OBJ(ai_platform_network_init(network, params));
  ai_bool ok = true;

  if (!net_ctx) return false;
  ok &= network_configure_weights(net_ctx, params);
  ok &= network_configure_activations(net_ctx, params);

  ok &= ai_platform_network_post_init(network);

  return ok;
}


AI_API_ENTRY
ai_i32 ai_network_run(
  ai_handle network, const ai_buffer* input, ai_buffer* output)
{
  return ai_platform_network_process(network, input, output);
}


AI_API_ENTRY
ai_i32 ai_network_forward(ai_handle network, const ai_buffer* input)
{
  return ai_platform_network_process(network, input, NULL);
}



#undef AI_NETWORK_MODEL_SIGNATURE
#undef AI_NET_OBJ_INSTANCE
#undef AI_TOOLS_DATE_TIME
#undef AI_TOOLS_COMPILE_TIME

