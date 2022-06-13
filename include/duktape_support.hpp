#pragma once

#include "duktape.h"

duk_context* duk_start();
static duk_ret_t duk_print(duk_context *ctx);
static duk_ret_t duk_add_pattern(duk_context *ctx) {