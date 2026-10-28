#include "terminal_ui.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <chrono>
#include <algorithm>

// ANSI colors
#define COLOR_RESET     "\033[0m"
#define COLOR_BOLD      "\033[1m"
#define COLOR_DIM       "\033[2m"
#define COLOR_RED       "\033[31m"
#define COLOR_GREEN     "\033[32m"
#define COLOR_YELLOW    "\033[33m"
#define COLOR_CYAN      "\033[36m"

#define PROGRESS_BAR_WIDTH          24
#define PROGRESS_BAR_WIDTH_NARROW   10      // used when the terminal has less than 100 columns
#define PROGRESS_FIXED_WIDTH        19      // live line without the bar and the stage text ("  ⠋  100%  " + "  00:00") + 1 column margin
#define RESULT_BOX_MAX_WIDTH    80
#define NON_TTY_REPRINT_S       30      // without a terminal, print the progress again every 30s

static bool is_tty = false;         // stdout is a terminal: redraw the live line
static bool use_color = false;
static bool use_unicode = false;

// the live progress line
static bool progress_active = false;
static std::string progress_stage = "";
static std::string progress_text = "";     // last printed text, for the non terminal mode
static int spinner_index = 0;
static std::chrono::steady_clock::time_point progress_start;
static std::chrono::steady_clock::time_point progress_last_print;

// stdout saved by uiMuteOutput()
static int saved_stdout = -1;

static const char* color(const char* code){
	return use_color ? code : "";
}

static const char* symbol(const char* unicode, const char* ascii){
	return use_unicode ? unicode : ascii;
}

/**
 * Width of the terminal in columns (80 if unknown)
 **/
static int terminalWidth(){
	struct winsize size;
	if(ioctl(STDOUT_FILENO, TIOCGWINSZ, &size) == 0 && size.ws_col > 0)
		return size.ws_col;
	return 80;
}

/**
 * Number of columns of an UTF-8 text (all symbols used here are 1 column wide)
 **/
static int displayWidth(const std::string& text){
	int width = 0;
	for(unsigned char c : text){
		if((c & 0xC0) != 0x80)  // not an UTF-8 continuation byte
			width++;
	}
	return width;
}

/**
 * Cut the text to <width> columns, end with "…" when it is cut
 **/
static std::string fitWidth(const std::string& text, int width){
	if(displayWidth(text) <= width)
		return text;
	if(width <= 1)
		return "";

	std::string result;
	int columns = 0;
	for(size_t i = 0; i < text.size(); i++){
		unsigned char c = text[i];
		if((c & 0xC0) != 0x80){
			if(columns == width - 1)
				break;
			columns++;
		}
		result += text[i];
	}
	return result + symbol("…", ".");
}

static int progressBarWidth(){
	return (terminalWidth() >= 100) ? PROGRESS_BAR_WIDTH : PROGRESS_BAR_WIDTH_NARROW;
}

static bool isUtf8Locale(){
	const char* names[] = {"LC_ALL", "LC_CTYPE", "LANG"};
	for(const char* name : names){
		const char* value = getenv(name);
		if(value != nullptr && value[0] != '\0')
			return strstr(value, "UTF-8") != nullptr || strstr(value, "utf8") != nullptr;
	}
	return false;
}

void uiInit(){
	const char* term = getenv("TERM");

	is_tty = isatty(STDOUT_FILENO);
	use_color = is_tty && getenv("NO_COLOR") == nullptr && !(term != nullptr && strcmp(term, "dumb") == 0);
	use_unicode = isUtf8Locale();
}

/**
 * Remove the live line before printing a normal line, uiProgress() draws it again later
 **/
static void clearLiveLine(){
	if(progress_active && is_tty)
		printf("\r\033[K");
}

static void printMark(const char* mark_color, const char* mark, const std::string& text){
	clearLiveLine();
	printf("  %s%s%s %s\n", color(mark_color), mark, color(COLOR_RESET), text.c_str());
	fflush(stdout);
}

void uiTitle(const std::string& title){
	std::string line;
	for(size_t i = 0; i < title.size() + 4; i++)
		line += symbol("─", "-");

	printf("\n%s%s%s%s%s\n", color(COLOR_CYAN), symbol("╭", "+"), line.c_str(), symbol("╮", "+"), color(COLOR_RESET));
	printf("%s%s%s  %s%s%s  %s%s%s\n", color(COLOR_CYAN), symbol("│", "|"), color(COLOR_RESET),
			color(COLOR_BOLD), title.c_str(), color(COLOR_RESET),
			color(COLOR_CYAN), symbol("│", "|"), color(COLOR_RESET));
	printf("%s%s%s%s%s\n\n", color(COLOR_CYAN), symbol("╰", "+"), line.c_str(), symbol("╯", "+"), color(COLOR_RESET));
}

void uiField(const std::string& name, const std::string& value){
	printf("  %s%-16s%s %s\n", color(COLOR_DIM), name.c_str(), color(COLOR_RESET), value.c_str());
}

void uiStep(int index, int total, const std::string& title){
	clearLiveLine();
	printf("\n%s[%d/%d]%s %s%s%s\n", color(COLOR_CYAN), index, total, color(COLOR_RESET),
			color(COLOR_BOLD), title.c_str(), color(COLOR_RESET));
	fflush(stdout);
}

void uiOk(const std::string& text){
	printMark(COLOR_GREEN, symbol("✔", "[OK]"), text);
}

void uiFail(const std::string& text){
	printMark(COLOR_RED, symbol("✖", "[FAIL]"), text);
}

void uiWarn(const std::string& text){
	printMark(COLOR_YELLOW, symbol("⚠", "[WARN]"), text);
}

void uiInfo(const std::string& text){
	clearLiveLine();
	printf("  %s%s%s\n", color(COLOR_DIM), text.c_str(), color(COLOR_RESET));
	fflush(stdout);
}

void uiPrompt(const std::string& label, const std::string& default_value){
	printf("  %s?%s %s", color(COLOR_CYAN), color(COLOR_RESET), label.c_str());
	if(!default_value.empty())
		printf(" %s(%s)%s", color(COLOR_DIM), default_value.c_str(), color(COLOR_RESET));
	printf(": ");
	fflush(stdout);
}

static std::string formatTime(long seconds){
	char text[16];
	snprintf(text, sizeof(text), "%02ld:%02ld", seconds / 60, seconds % 60);
	return text;
}

void uiProgress(int percent, const std::string& stage, const std::string& detail){
	auto now = std::chrono::steady_clock::now();

	if(!progress_active){
		progress_active = true;
		progress_start = now;
		progress_last_print = now;
		progress_stage = "";
		progress_text = "";
	}

	// the previous stage is done
	if(!progress_stage.empty() && stage != progress_stage){
		std::string done_stage = progress_stage;
		progress_stage = stage;     // set first: uiOk() must not print it again
		uiOk(done_stage);
	}
	progress_stage = stage;

	if(percent < 0) percent = 0;
	if(percent > 100) percent = 100;
	long elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - progress_start).count();
	std::string text = stage + (detail.empty() ? "" : std::string(" ") + symbol("·", "-") + " " + detail);
	if(is_tty){
		// keep the live line on one row: a wrapped line can not be redrawn with '\r'
		int text_width = terminalWidth() - PROGRESS_FIXED_WIDTH - progressBarWidth();
		text = fitWidth(text, (text_width < 10) ? 10 : text_width);
	}

	if(is_tty){
		const char* spinner_unicode[] = {"⠋", "⠙", "⠹", "⠸", "⠼", "⠴", "⠦", "⠧", "⠇", "⠏"};
		const char* spinner_ascii[] = {"|", "/", "-", "\\"};
		const char* spinner = use_unicode ? spinner_unicode[spinner_index % 10] : spinner_ascii[spinner_index % 4];
		spinner_index++;

		int bar_width = progressBarWidth();
		int filled = percent * bar_width / 100;
		std::string bar;
		for(int i = 0; i < bar_width; i++)
			bar += (i < filled) ? symbol("█", "#") : symbol("░", "-");

		printf("\r\033[K  %s%s%s %s%s%s %3d%%  %s  %s%s%s",
				color(COLOR_CYAN), spinner, color(COLOR_RESET),
				color(COLOR_CYAN), bar.c_str(), color(COLOR_RESET),
				percent, text.c_str(),
				color(COLOR_DIM), formatTime(elapsed).c_str(), color(COLOR_RESET));
		fflush(stdout);
		return;
	}

	// not a terminal (pipe / file): print on a new stage, every 10% or every NON_TTY_REPRINT_S
	long since_print = std::chrono::duration_cast<std::chrono::seconds>(now - progress_last_print).count();
	std::string step_key = stage + "|" + std::to_string(percent / 10);
	if(step_key != progress_text || since_print >= NON_TTY_REPRINT_S){
		printf("  [%3d%%] %s (%s)\n", percent, text.c_str(), formatTime(elapsed).c_str());
		fflush(stdout);
		progress_text = step_key;
		progress_last_print = now;
	}
}

void uiProgressClear(){
	clearLiveLine();
	fflush(stdout);
	progress_active = false;
	progress_stage = "";
}

void uiProgressEnd(bool is_ok){
	if(!progress_active)
		return;

	clearLiveLine();
	std::string last_stage = progress_stage;
	progress_active = false;
	progress_stage = "";

	if(last_stage.empty())
		return;
	if(is_ok)
		uiOk(last_stage);
	else
		uiFail(last_stage);
}

/**
 * One row of the result box: "│ <text><spaces> │"
 **/
static void printBoxRow(const char* box_color, const std::string& text, const std::string& text_color, int inner_width){
	std::string fitted = fitWidth(text, inner_width);
	int padding = inner_width - displayWidth(fitted);
	printf("%s%s%s %s%s%s%s %s%s%s\n", color(box_color), symbol("│", "|"), color(COLOR_RESET),
			text_color.c_str(), fitted.c_str(), color(COLOR_RESET), std::string(padding, ' ').c_str(),
			color(box_color), symbol("│", "|"), color(COLOR_RESET));
}

static void printBoxLine(const char* box_color, const char* left, const char* right, int inner_width){
	std::string line;
	for(int i = 0; i < inner_width + 2; i++)
		line += symbol("─", "-");
	printf("%s%s%s%s%s\n", color(box_color), left, line.c_str(), right, color(COLOR_RESET));
}

void uiResult(bool is_ok, const std::string& title,
              const std::vector<std::pair<std::string, std::string>>& fields,
              const std::vector<std::string>& notes){
	const char* box_color = is_ok ? COLOR_GREEN : COLOR_RED;
	std::string heading = std::string(is_ok ? symbol("✔", "[OK]") : symbol("✖", "[FAIL]")) + " " + title;

	// rows of the box
	std::vector<std::string> rows;
	for(const auto& field : fields){
		std::string name = field.first;
		name.resize(14, ' ');
		rows.push_back(name + " " + field.second);
	}

	// box width: the longest row, limited by the terminal width
	int inner_width = displayWidth(heading);
	for(const std::string& row : rows)
		inner_width = std::max(inner_width, displayWidth(row));
	for(const std::string& note : notes)
		inner_width = std::max(inner_width, displayWidth(note));
	int max_width = std::min(terminalWidth() - 1, RESULT_BOX_MAX_WIDTH) - 4;   // "│ " + " │", 1 column margin
	if(inner_width > max_width)
		inner_width = max_width;

	printf("\n");
	printBoxLine(box_color, symbol("┌", "+"), symbol("┐", "+"), inner_width);
	printBoxRow(box_color, heading, std::string(color(COLOR_BOLD)) + color(box_color), inner_width);
	printBoxLine(box_color, symbol("├", "+"), symbol("┤", "+"), inner_width);
	for(const std::string& row : rows)
		printBoxRow(box_color, row, "", inner_width);
	for(const std::string& note : notes)
		printBoxRow(box_color, note, color(COLOR_YELLOW), inner_width);
	printBoxLine(box_color, symbol("└", "+"), symbol("┘", "+"), inner_width);
	printf("\n");
	fflush(stdout);
}

void uiMuteOutput(){
	fflush(stdout);
	fflush(stderr);

	int null_fd = open("/dev/null", O_WRONLY);
	if(null_fd < 0)
		return;

	if(saved_stdout < 0)
		saved_stdout = dup(STDOUT_FILENO);
	dup2(null_fd, STDOUT_FILENO);
	dup2(null_fd, STDERR_FILENO);
	close(null_fd);
}

void uiRestoreStdout(){
	if(saved_stdout < 0)
		return;

	fflush(stdout);
	dup2(saved_stdout, STDOUT_FILENO);
	close(saved_stdout);
	saved_stdout = -1;
}

std::string uiFormatSize(long bytes){
	char text[32];
	if(bytes >= 1024L * 1024 * 1024)
		snprintf(text, sizeof(text), "%.2f GB", bytes / (1024.0 * 1024 * 1024));
	else if(bytes >= 1024L * 1024)
		snprintf(text, sizeof(text), "%.1f MB", bytes / (1024.0 * 1024));
	else if(bytes >= 1024)
		snprintf(text, sizeof(text), "%.1f KB", bytes / 1024.0);
	else
		snprintf(text, sizeof(text), "%ld B", bytes);
	return text;
}
