/**
 * Simple terminal UI for the update example:
 *   - steps "[1/5] ...", result marks ✔ ✖ ⚠
 *   - one live progress line: spinner, bar, percent, stage, elapsed time
 *   - colors only on a real terminal (disabled by NO_COLOR, TERM=dumb or when the output is a pipe/file)
 *   - unicode symbols only when the locale is UTF-8
 **/

#ifndef TERMINAL_UI_H
#define TERMINAL_UI_H

#include <string>
#include <vector>
#include <utility>

void uiInit();

// header and steps
void uiTitle(const std::string& title);
void uiField(const std::string& name, const std::string& value);
void uiStep(int index, int total, const std::string& title);

// one line messages
void uiOk(const std::string& text);
void uiFail(const std::string& text);
void uiWarn(const std::string& text);
void uiInfo(const std::string& text);
void uiPrompt(const std::string& label, const std::string& default_value);

/**
 * Live progress line. Call it often (each poll): it redraws the spinner and the time.
 * When <stage> changes, the previous stage is printed as done (✔) above the live line.
 * detail: extra text after the stage, e.g. "flash 63%", "offline"
 **/
void uiProgress(int percent, const std::string& stage, const std::string& detail);

// End the live line: print the last stage as done (✔) or failed (✖)
void uiProgressEnd(bool is_ok);

// Remove the live line without printing anything (the caller prints its own result)
void uiProgressClear();

/**
 * Final box with the result
 **/
void uiResult(bool is_ok, const std::string& title,
              const std::vector<std::pair<std::string, std::string>>& fields,
              const std::vector<std::string>& notes);

// Hide the log of the PayloadSdk library (it prints its debug messages to stdout/stderr)
void uiMuteOutput();
void uiRestoreStdout();     // stderr stays muted: the SDK threads may still print warnings

std::string uiFormatSize(long bytes);

#endif // TERMINAL_UI_H
