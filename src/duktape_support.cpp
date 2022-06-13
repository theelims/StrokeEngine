#include "esp_log.h"
#include "duktape_support.hpp"
#include "pattern.h"

duk_context* duk_start() {
  duk_context* ctx = duk_create_heap_default();
  
  duk_push_c_function(ctx, duk_print, 1 /*nargs*/);
  duk_put_global_string(ctx, "print");

  duk_push_global_stash(ctx);
  duk_push_object(ctx);
  duk_put_prop_string(ctx, -2, "patterns");

  duk_pop(ctx);

  return ctx;
}

static duk_ret_t duk_print(duk_context *ctx) {
  ESP_LOGI("DukTape", "%s", duk_to_string(ctx, 0));
  return 0;  /* no return value (= undefined) */
}

static duk_ret_t duk_add_pattern(duk_context *ctx) {
  // [name, state, callback]
  const char *name = duk_require_string(ctx, -3);
  duk_require_object(ctx, -2);
  duk_require_function(ctx, -1);

  duk_push_global_stash(ctx);
  // [name, state, callback, gStash]

  duk_get_prop_string(ctx, -1, "patterns");
  // [name, state, callback, gStash, patterns]
  
  // Create a new Pattern object in JS
  duk_push_object(ctx);

  duk_dup(ctx, -5);
  duk_put_prop_string(ctx, -1, "initialState");
  
  duk_push_object(ctx);
  duk_put_prop_string(ctx, -1, "state");

  duk_dup(ctx, -4);
  duk_put_prop_string(ctx, -1, "callback");

  duk_put_prop_string(ctx, -2, name);

  // [name, state, callback, gStash, patterns]
  duk_pop_n(ctx, 5);

  Pattern::register(name);
}

void duk_init_pattern(duk_context *ctx, const char *name) {
  duk_push_global_stash(ctx);
  duk_get_prop_string(ctx, -1, "patterns");
  duk_get_prop_string(ctx, -1, name);
  duk_get_prop_string(ctx, -1, "initialState");
  
  // Create a new blank object which inherits from initialState
  duk_push_object(ctx);
  duk_set_prototype(ctx, -2);
  duk_put_prop_string(ctx, -2, "state");

  duk_pop_n(ctx, 4);
}

void duk_call_pattern(duk_context *ctx, const char *name) {
  duk_push_global_stash(ctx);
  duk_get_prop_string(ctx, -1, "patterns");
  duk_get_prop_string(ctx, -1, name);

  duk_get_prop_string(ctx, -1, "callback"); // func
  duk_get_prop_string(ctx, -1, "state"); // this

  duk_push_uint(ctx, 0); // timestamp
  duk_push_boolean(ctx, false); // isAtTarget

  // Parameters
  duk_push_object(ctx);

  duk_push_number(ctx, 0);
  duk_put_prop_string(ctx, -1, "stroke");
  
  duk_push_number(ctx, 0);
  duk_put_prop_string(ctx, -1, "sensation");
  // TODO - parameters

  duk_call_method(ctx, 3);

  duk_get_prop_string(ctx, -1, "position");
  double position = duk_get_number(ctx, -1);
  duk_pop(ctx);

  duk_get_prop_string(ctx, -1, "speed");
  double position = duk_get_number(ctx, -1);
  duk_pop(ctx);

  duk_get_prop_string(ctx, -1, "acceleration");
  double position = duk_get_number(ctx, -1);
  duk_pop_n(ctx, 5);
}