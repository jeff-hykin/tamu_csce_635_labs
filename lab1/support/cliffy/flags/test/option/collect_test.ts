import { assertEquals, assertThrows } from "../../../dev_deps.ts";
import { OptionType } from "../../deprecated.ts";
import { parseFlags } from "../../flags.ts";
import type { ParseFlagsOptions } from "../../types.ts";

const options: ParseFlagsOptions = {
  allowEmpty: false,
  flags: [{
    name: "flag",
    aliases: ["f"],
    type: OptionType.STRING,
    optionalValue: true,
  }, {
    name: "no-flag",
  }, {
    name: "string",
    aliases: ["s"],
    type: OptionType.STRING,
    collect: true,
  }, {
    name: "boolean",
    aliases: ["b"],
    type: OptionType.BOOLEAN,
    optionalValue: true,
    collect: true,
  }, {
    name: "number",
    aliases: ["n"],
    type: OptionType.NUMBER,
    collect: true,
  }],
};

Deno.test("flags - option - collect - short flag can only occur once", () => {
  assertThrows(
    () => parseFlags(["-f", "-f"], options),
    Error,
    `Option "-f" can only occur once, but was found several times.`,
  );
});

Deno.test("flags - option - collect - long flag can only occur once", () => {
  assertThrows(
    () => parseFlags(["-f", "--flag"], options),
    Error,
    `Option "--flag" can only occur once, but was found several times.`,
  );
});

Deno.test("flags - option - collect - long and short flag can not occur together", () => {
  assertThrows(
    () => parseFlags(["-f", "true", "--flag", "false"], options),
    Error,
    `Option "--flag" can only occur once, but was found several times.`,
  );
});

Deno.test("flags - option - collect - flag and negatable flag can not occur together", () => {
  assertThrows(
    () => parseFlags(["-f", "--no-flag"], options),
    Error,
    `Option "--flag" can only occur once, but was found several times.`,
  );
});

Deno.test("flags - option - collect - collect boolean value", () => {
  const { flags, unknown, literal } = parseFlags(
    ["-b", "1", "--boolean", "0"],
    options,
  );

  assertEquals(flags, { boolean: [true, false] });
  assertEquals(unknown, []);
  assertEquals(literal, []);
});

Deno.test("flags - option - collect - collect string value", () => {
  const { flags, unknown, literal } = parseFlags(
    ["-s", "1", "--string", "0"],
    options,
  );

  assertEquals(flags, { string: ["1", "0"] });
  assertEquals(unknown, []);
  assertEquals(literal, []);
});

Deno.test("flags - option - collect - collect number value", () => {
  const { flags, unknown, literal } = parseFlags(
    ["-n", "1", "--number", "0"],
    options,
  );

  assertEquals(flags, { number: [1, 0] });
  assertEquals(unknown, []);
  assertEquals(literal, []);
});

Deno.test("flags - option - collect - collect values by default", () => {
  const { flags, unknown, literal } = parseFlags([
    "--foo",
    "1",
    "--foo",
    "2",
    "--foo",
    "3",
  ]);

  assertEquals(flags, { foo: ["1", "2", "3"] });
  assertEquals(unknown, []);
  assertEquals(literal, []);
});
