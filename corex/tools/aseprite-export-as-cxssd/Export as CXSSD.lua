-- Exports an Aseprite file to CXSSD, a custom file format for the CoreX engine.
-- Script by Sean Francis N. Ballais (@seanballais | https::seanballais.github.io)

--
-- json.lua
--
-- Copyright (c) 2020 rxi
--
-- Permission is hereby granted, free of charge, to any person obtaining a copy of
-- this software and associated documentation files (the "Software"), to deal in
-- the Software without restriction, including without limitation the rights to
-- use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
-- of the Software, and to permit persons to whom the Software is furnished to do
-- so, subject to the following conditions:
--
-- The above copyright notice and this permission notice shall be included in all
-- copies or substantial portions of the Software.
--
-- THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
-- IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
-- FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
-- AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
-- LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
-- OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
-- SOFTWARE.
--

local json = { _version = "0.1.2" }

-------------------------------------------------------------------------------
-- Encode
-------------------------------------------------------------------------------

local encode

local escape_char_map = {
  [ "\\" ] = "\\",
  [ "\"" ] = "\"",
  [ "\b" ] = "b",
  [ "\f" ] = "f",
  [ "\n" ] = "n",
  [ "\r" ] = "r",
  [ "\t" ] = "t",
}

local escape_char_map_inv = { [ "/" ] = "/" }
for k, v in pairs(escape_char_map) do
  escape_char_map_inv[v] = k
end


local function escape_char(c)
  return "\\" .. (escape_char_map[c] or string.format("u%04x", c:byte()))
end


local function encode_nil(val)
  return "null"
end


local function encode_table(val, stack)
  local res = {}
  stack = stack or {}

  -- Circular reference?
  if stack[val] then error("circular reference") end

  stack[val] = true

  if rawget(val, 1) ~= nil or next(val) == nil then
    -- Treat as array -- check keys are valid and it is not sparse
    local n = 0
    for k in pairs(val) do
      if type(k) ~= "number" then
        error("invalid table: mixed or invalid key types")
      end
      n = n + 1
    end
    if n ~= #val then
      error("invalid table: sparse array")
    end
    -- Encode
    for i, v in ipairs(val) do
      table.insert(res, encode(v, stack))
    end
    stack[val] = nil
    return "[" .. table.concat(res, ",") .. "]"

  else
    -- Treat as an object
    for k, v in pairs(val) do
      if type(k) ~= "string" then
        error("invalid table: mixed or invalid key types")
      end
      table.insert(res, encode(k, stack) .. ":" .. encode(v, stack))
    end
    stack[val] = nil
    return "{" .. table.concat(res, ",") .. "}"
  end
end


local function encode_string(val)
  return '"' .. val:gsub('[%z\1-\31\\"]', escape_char) .. '"'
end


local function encode_number(val)
  -- Check for NaN, -inf and inf
  if val ~= val or val <= -math.huge or val >= math.huge then
    error("unexpected number value '" .. tostring(val) .. "'")
  end
  return string.format("%.14g", val)
end


local type_func_map = {
  [ "nil"     ] = encode_nil,
  [ "table"   ] = encode_table,
  [ "string"  ] = encode_string,
  [ "number"  ] = encode_number,
  [ "boolean" ] = tostring,
}


encode = function(val, stack)
  local t = type(val)
  local f = type_func_map[t]
  if f then
    return f(val, stack)
  end
  error("unexpected type '" .. t .. "'")
end


function json.encode(val)
  return ( encode(val) )
end


-------------------------------------------------------------------------------
-- Decode
-------------------------------------------------------------------------------

local parse

local function create_set(...)
  local res = {}
  for i = 1, select("#", ...) do
    res[ select(i, ...) ] = true
  end
  return res
end

local space_chars   = create_set(" ", "\t", "\r", "\n")
local delim_chars   = create_set(" ", "\t", "\r", "\n", "]", "}", ",")
local escape_chars  = create_set("\\", "/", '"', "b", "f", "n", "r", "t", "u")
local literals      = create_set("true", "false", "null")

local literal_map = {
  [ "true"  ] = true,
  [ "false" ] = false,
  [ "null"  ] = nil,
}


local function next_char(str, idx, set, negate)
  for i = idx, #str do
    if set[str:sub(i, i)] ~= negate then
      return i
    end
  end
  return #str + 1
end


local function decode_error(str, idx, msg)
  local line_count = 1
  local col_count = 1
  for i = 1, idx - 1 do
    col_count = col_count + 1
    if str:sub(i, i) == "\n" then
      line_count = line_count + 1
      col_count = 1
    end
  end
  error( string.format("%s at line %d col %d", msg, line_count, col_count) )
end


local function codepoint_to_utf8(n)
  -- http://scripts.sil.org/cms/scripts/page.php?site_id=nrsi&id=iws-appendixa
  local f = math.floor
  if n <= 0x7f then
    return string.char(n)
  elseif n <= 0x7ff then
    return string.char(f(n / 64) + 192, n % 64 + 128)
  elseif n <= 0xffff then
    return string.char(f(n / 4096) + 224, f(n % 4096 / 64) + 128, n % 64 + 128)
  elseif n <= 0x10ffff then
    return string.char(f(n / 262144) + 240, f(n % 262144 / 4096) + 128,
                       f(n % 4096 / 64) + 128, n % 64 + 128)
  end
  error( string.format("invalid unicode codepoint '%x'", n) )
end


local function parse_unicode_escape(s)
  local n1 = tonumber( s:sub(1, 4),  16 )
  local n2 = tonumber( s:sub(7, 10), 16 )
   -- Surrogate pair?
  if n2 then
    return codepoint_to_utf8((n1 - 0xd800) * 0x400 + (n2 - 0xdc00) + 0x10000)
  else
    return codepoint_to_utf8(n1)
  end
end


local function parse_string(str, i)
  local res = ""
  local j = i + 1
  local k = j

  while j <= #str do
    local x = str:byte(j)

    if x < 32 then
      decode_error(str, j, "control character in string")

    elseif x == 92 then -- `\`: Escape
      res = res .. str:sub(k, j - 1)
      j = j + 1
      local c = str:sub(j, j)
      if c == "u" then
        local hex = str:match("^[dD][89aAbB]%x%x\\u%x%x%x%x", j + 1)
                 or str:match("^%x%x%x%x", j + 1)
                 or decode_error(str, j - 1, "invalid unicode escape in string")
        res = res .. parse_unicode_escape(hex)
        j = j + #hex
      else
        if not escape_chars[c] then
          decode_error(str, j - 1, "invalid escape char '" .. c .. "' in string")
        end
        res = res .. escape_char_map_inv[c]
      end
      k = j + 1

    elseif x == 34 then -- `"`: End of string
      res = res .. str:sub(k, j - 1)
      return res, j + 1
    end

    j = j + 1
  end

  decode_error(str, i, "expected closing quote for string")
end


local function parse_number(str, i)
  local x = next_char(str, i, delim_chars)
  local s = str:sub(i, x - 1)
  local n = tonumber(s)
  if not n then
    decode_error(str, i, "invalid number '" .. s .. "'")
  end
  return n, x
end


local function parse_literal(str, i)
  local x = next_char(str, i, delim_chars)
  local word = str:sub(i, x - 1)
  if not literals[word] then
    decode_error(str, i, "invalid literal '" .. word .. "'")
  end
  return literal_map[word], x
end


local function parse_array(str, i)
  local res = {}
  local n = 1
  i = i + 1
  while 1 do
    local x
    i = next_char(str, i, space_chars, true)
    -- Empty / end of array?
    if str:sub(i, i) == "]" then
      i = i + 1
      break
    end
    -- Read token
    x, i = parse(str, i)
    res[n] = x
    n = n + 1
    -- Next token
    i = next_char(str, i, space_chars, true)
    local chr = str:sub(i, i)
    i = i + 1
    if chr == "]" then break end
    if chr ~= "," then decode_error(str, i, "expected ']' or ','") end
  end
  return res, i
end


local function parse_object(str, i)
  local res = {}
  i = i + 1
  while 1 do
    local key, val
    i = next_char(str, i, space_chars, true)
    -- Empty / end of object?
    if str:sub(i, i) == "}" then
      i = i + 1
      break
    end
    -- Read key
    if str:sub(i, i) ~= '"' then
      decode_error(str, i, "expected string for key")
    end
    key, i = parse(str, i)
    -- Read ':' delimiter
    i = next_char(str, i, space_chars, true)
    if str:sub(i, i) ~= ":" then
      decode_error(str, i, "expected ':' after key")
    end
    i = next_char(str, i + 1, space_chars, true)
    -- Read value
    val, i = parse(str, i)
    -- Set
    res[key] = val
    -- Next token
    i = next_char(str, i, space_chars, true)
    local chr = str:sub(i, i)
    i = i + 1
    if chr == "}" then break end
    if chr ~= "," then decode_error(str, i, "expected '}' or ','") end
  end
  return res, i
end


local char_func_map = {
  [ '"' ] = parse_string,
  [ "0" ] = parse_number,
  [ "1" ] = parse_number,
  [ "2" ] = parse_number,
  [ "3" ] = parse_number,
  [ "4" ] = parse_number,
  [ "5" ] = parse_number,
  [ "6" ] = parse_number,
  [ "7" ] = parse_number,
  [ "8" ] = parse_number,
  [ "9" ] = parse_number,
  [ "-" ] = parse_number,
  [ "t" ] = parse_literal,
  [ "f" ] = parse_literal,
  [ "n" ] = parse_literal,
  [ "[" ] = parse_array,
  [ "{" ] = parse_object,
}


parse = function(str, idx)
  local chr = str:sub(idx, idx)
  local f = char_func_map[chr]
  if f then
    return f(str, idx)
  end
  decode_error(str, idx, "unexpected character '" .. chr .. "'")
end


function json.decode(str)
  if type(str) ~= "string" then
    error("expected argument of type string, got " .. type(str))
  end
  local res, idx = parse(str, next_char(str, 1, space_chars, true))
  idx = next_char(str, idx, space_chars, true)
  if idx <= #str then
    decode_error(str, idx, "trailing garbage")
  end
  return res
end

-- Script code starts here.

local function exportSpritesheet(dialog)
    local dialog_data = dialog.data

    if dialog_data.output_file ~= "" then
        path = app.fs.filePath(dialog_data.output_file)
        file_title = app.fs.fileTitle(dialog_data.output_file)
        extension = app.fs.fileExtension(dialog_data.output_file)

        -- Ask for confirmation.
        local should_proceed = app.alert{ title="Warning",
                                          text="Are you sure you want to export?",
                                          buttons={ "Yes", "No" } }
        if should_proceed == 2 then
            return
        end

        -- Export spritesheet image.
        texture_file_name = file_title .. ".png"
        json_file_name = file_title .. ".json"
        texture_file = app.fs.joinPath(path, texture_file_name)
        json_file = app.fs.joinPath(path, json_file_name)

        local sheet_type
        if dialog_data.sheet_type == "Horizontal Strip" then
            sheet_type = SpriteSheetType.HORIZONTAL
        elseif dialog_data.sheet_type == "Vertical Strip" then
            sheet_type = SpriteSheetType.VERTICAL
        elseif dialog_data.sheet_type == "By Rows" then
            sheet_type = SpriteSheetType.ROWS
        elseif dialog_data.sheet_type == "By Columns" then
            sheet_type = SpriteSheetType.COLUMNS
        elseif dialog_data.sheet_type == "Packed" then
            sheet_type = SpriteSheetType.PACKED
        end

        app.command.ExportSpriteSheet {
            ui=false,
            askOverwrite=false,
            type=sheet_type,
            textureFilename=texture_file,
            dataFilename=json_file,
            dataFormat=SpriteSheetDataFormat.JSON_ARRAY
        }

        -- Export CXSSD file.
        csxxd_file_name = file_title .. ".cxssd"
        cxssd_file = app.fs.joinPath(path, csxxd_file_name)

        json_file_name = file_title .. ".json"
        json_file = app.fs.joinPath(path, json_file_name)

        ---- Handle spritesheet data
        aseprite_data_file = io.open(json_file, "rb")
        aseprite_data = json.decode(aseprite_data_file:read("*all"))
        aseprite_data_file:close()

        data = {}

        ---- Handle frames
        has_gotten_frame_duration = false
        data["frames"] = {}
        for i, frame_data in ipairs(aseprite_data["frames"]) do
            data["frames"][i] = frame_data["frame"]

            if not has_gotten_frame_duration then
                data["time_per_frame"] = frame_data["duration"] / 1000
                has_gotten_frame_duration = true
            end
        end

        ---- Handle tags
        data["custom_states"] = {}
        data["custom_states"][1] = {
            ["name"] = "all",
            ["start_frame"] = 0,
            ["end_frame"] = #app.activeSprite.frames,
            ["is_looped"] = dialog_data.all_frames_looped
        }

        for i, tag in ipairs(aseprite_data["meta"]["frameTags"], 2) do
            data["custom_states"][i] = {
                ["name"] = tag["name"],
                ["start_frame"] = tag["from"],
                ["end_frame"] = tag["to"],
                ["is_looped"] = dialog_data["tag_looped_" .. i]
            }
        end

        output_cxssd_file = io.open(cxssd_file, "w")
        
        -- Append a new line for POSIX compliance.
        output_cxssd_file:write(json.encode(data) .. "\n")

        output_cxssd_file:close()

        app.alert{ title="CXSSD Export Status",
                   text="Export completed successfully. Please delete the JSON file in the export folder." }
        dialog:close()
    else
        app.alert{ title="CXSSD Export Error",
                   text="Error: Output file is not specified." }
    end
end

if not app.activeSprite then
    return app.alert "There is no active sprite."
end

local dialog = Dialog("Export to CoreX Spritesheet Data")

dialog:separator{ text="General" }
      :newrow()
      :combobox{ id="sheet_type",
                 label="Sheet Type:",
                 options={ "Horizontal Strip",
                           "Vertical Strip",
                           "By Rows",
                           "By Columns",
                           "Packed" } }
      :newrow()
      :combobox{ id="frame_collection",
                 label="Frames [UNIMPLEMENTED]:",
                 options={ "All Frames", "Selected Frames" } }
      :newrow()
      :separator{ text="Tag Information" }
      :newrow()

dialog:label{ label="All Frames" }
      :newrow()
      :check { id="all_frames_looped",
               label="  Set to Looped:" }

for i,tag in ipairs(app.activeSprite.tags) do
    dialog:label{ label=tag.name }
          :newrow()
          :check { id="tag_looped_" .. i,
                   label="  Set to Looped:" }
end

dialog:separator{ text="Export Details" }
      :newrow()
      :file { id="output_file",
              label="Output File:",
              save=true,
              filetypes={ "png", "jpeg" },
              entry=true }
      :newrow()
      :button{ text="Export", onclick=function() exportSpritesheet(dialog) end }

dialog:show()
