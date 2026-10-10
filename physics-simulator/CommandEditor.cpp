#include "CommandEditor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

// Creates a command editor inside the specified screen rectangle.
CommandEditor::CommandEditor(
    Rectangle editorBounds,
    int editorFontSize
)
    : bounds(editorBounds),
      text(""),
      cursorPosition(0),
      selectionAnchor(0),
      active(false),
      mouseSelecting(false),
      scrollX(0.0f),
      scrollY(0.0f),
      fontSize(editorFontSize),
      padding(10.0f) {
}

// Returns the vertical distance between consecutive text lines.
float CommandEditor::getLineHeight() const {
    return static_cast<float>(fontSize + 5);
}

// Returns the number of logical lines in the current text.
int CommandEditor::getLineCount() const {

    int lines = 1;

    for (char character : text) {
        if (character == '\n') {
            ++lines;
        }
    }

    return lines;
}

// Returns the largest line width in pixels.
float CommandEditor::getMaxLineWidth() const {

    Font font = GetFontDefault();

    float maxWidth = 0.0f;

    std::size_t lineStart = 0;

    while (lineStart <= text.size()) {

        std::size_t lineEnd =
            text.find('\n', lineStart);

        if (lineEnd == std::string::npos) {
            lineEnd = text.size();
        }

        std::string line =
            text.substr(
                lineStart,
                lineEnd - lineStart
            );

        float width =
            MeasureTextEx(
                font,
                line.c_str(),
                static_cast<float>(fontSize),
                1.0f
            ).x;

        maxWidth =
            std::max(
                maxWidth,
                width
            );

        if (lineEnd == text.size()) {
            break;
        }

        lineStart = lineEnd + 1;
    }

    return maxWidth;
}

// Returns the index where the line containing the cursor begins.
std::size_t CommandEditor::getCurrentLineStart() const {

    if (cursorPosition == 0) {
        return 0;
    }

    std::size_t position =
        text.rfind(
            '\n',
            cursorPosition - 1
        );

    if (position == std::string::npos) {
        return 0;
    }

    return position + 1;
}

// Returns the index where the line containing the cursor ends.
std::size_t CommandEditor::getCurrentLineEnd() const {

    std::size_t position =
        text.find(
            '\n',
            cursorPosition
        );

    if (position == std::string::npos) {
        return text.size();
    }

    return position;
}

// Returns the zero-based line number containing the cursor.
int CommandEditor::getCursorLine() const {

    int line = 0;

    for (
        std::size_t i = 0;
        i < cursorPosition;
        ++i
    ) {

        if (text[i] == '\n') {
            ++line;
        }
    }

    return line;
}

// Returns whether a text range is currently selected.
bool CommandEditor::hasSelection() const {
    return cursorPosition != selectionAnchor;
}

// Returns the first index of the current selection.
std::size_t CommandEditor::getSelectionStart() const {

    return std::min(
        cursorPosition,
        selectionAnchor
    );
}

// Returns the index immediately after the current selection.
std::size_t CommandEditor::getSelectionEnd() const {

    return std::max(
        cursorPosition,
        selectionAnchor
    );
}

// Deletes the currently selected text.
void CommandEditor::deleteSelection() {

    if (!hasSelection()) {
        return;
    }

    std::size_t start =
        getSelectionStart();

    std::size_t end =
        getSelectionEnd();

    text.erase(
        start,
        end - start
    );

    cursorPosition = start;
    selectionAnchor = start;
}

// Converts a mouse position into the nearest text index.
std::size_t CommandEditor::getPositionFromMouse(
    const Vector2& mousePosition
) const {

    float lineHeight =
        getLineHeight();

    float localY =
        mousePosition.y -
        bounds.y -
        padding +
        scrollY;

    int targetLine =
        static_cast<int>(
            std::floor(
                localY / lineHeight
            )
        );

    targetLine =
        std::max(
            0,
            targetLine
        );

    int lineNumber = 0;

    std::size_t lineStart = 0;

    while (
        lineNumber < targetLine &&
        lineStart < text.size()
    ) {

        std::size_t next =
            text.find(
                '\n',
                lineStart
            );

        if (next == std::string::npos) {
            lineStart = text.size();
            break;
        }

        lineStart = next + 1;
        ++lineNumber;
    }

    std::size_t lineEnd =
        text.find(
            '\n',
            lineStart
        );

    if (lineEnd == std::string::npos) {
        lineEnd = text.size();
    }

    float localX =
        mousePosition.x -
        bounds.x -
        padding +
        scrollX;

    Font font =
        GetFontDefault();

    std::size_t bestPosition =
        lineStart;

    float bestDistance =
        std::numeric_limits<float>::max();

    for (
        std::size_t i = lineStart;
        i <= lineEnd;
        ++i
    ) {

        std::string portion =
            text.substr(
                lineStart,
                i - lineStart
            );

        float width =
            MeasureTextEx(
                font,
                portion.c_str(),
                static_cast<float>(fontSize),
                1.0f
            ).x;

        float distance =
            std::abs(
                localX - width
            );

        if (distance < bestDistance) {

            bestDistance = distance;
            bestPosition = i;
        }
    }

    return bestPosition;
}

// Moves the cursor one line upward or downward.
void CommandEditor::moveCursorVertical(
    int direction
) {

    std::size_t currentStart =
        getCurrentLineStart();

    std::size_t column =
        cursorPosition -
        currentStart;

    if (direction < 0) {

        if (currentStart == 0) {
            return;
        }

        std::size_t previousEnd =
            currentStart - 1;

        std::size_t previousStart = 0;

        if (previousEnd > 0) {

            std::size_t separator =
                text.rfind(
                    '\n',
                    previousEnd - 1
                );

            if (
                separator !=
                std::string::npos
            ) {
                previousStart =
                    separator + 1;
            }
        }

        std::size_t previousLength =
            previousEnd -
            previousStart;

        cursorPosition =
            previousStart +
            std::min(
                column,
                previousLength
            );
    }
    else {

        std::size_t currentEnd =
            getCurrentLineEnd();

        if (currentEnd >= text.size()) {
            return;
        }

        std::size_t nextStart =
            currentEnd + 1;

        std::size_t nextEnd =
            text.find(
                '\n',
                nextStart
            );

        if (
            nextEnd ==
            std::string::npos
        ) {
            nextEnd =
                text.size();
        }

        std::size_t nextLength =
            nextEnd -
            nextStart;

        cursorPosition =
            nextStart +
            std::min(
                column,
                nextLength
            );
    }
}

// Keeps the cursor visible by adjusting horizontal and vertical scroll.
void CommandEditor::ensureCursorVisible() {

    Font font =
        GetFontDefault();

    float lineHeight =
        getLineHeight();

    float visibleWidth =
        bounds.width -
        2.0f * padding;

    float visibleHeight =
        bounds.height -
        2.0f * padding;

    std::size_t lineStart =
        getCurrentLineStart();

    std::string beforeCursor =
        text.substr(
            lineStart,
            cursorPosition -
            lineStart
        );

    float cursorX =
        MeasureTextEx(
            font,
            beforeCursor.c_str(),
            static_cast<float>(fontSize),
            1.0f
        ).x;

    const float horizontalMargin = 8.0f;

    if (cursorX < scrollX) {

        scrollX = cursorX;
    }

    if (
        cursorX >
        scrollX +
        visibleWidth -
        horizontalMargin
    ) {

        scrollX =
            cursorX -
            visibleWidth +
            horizontalMargin;
    }

    float maxScrollX =
        std::max(
            0.0f,
            getMaxLineWidth() -
            visibleWidth
        );

    scrollX =
        std::clamp(
            scrollX,
            0.0f,
            maxScrollX
        );

    float cursorTop =
        getCursorLine() *
        lineHeight;

    if (cursorTop < scrollY) {
        scrollY = cursorTop;
    }

    if (
        cursorTop +
        lineHeight >
        scrollY +
        visibleHeight
    ) {

        scrollY =
            cursorTop +
            lineHeight -
            visibleHeight;
    }

    float contentHeight =
        getLineCount() *
        lineHeight;

    float maxScrollY =
        std::max(
            0.0f,
            contentHeight -
            visibleHeight
        );

    scrollY =
        std::clamp(
            scrollY,
            0.0f,
            maxScrollY
        );
}

// Inserts text at the cursor, replacing any active selection.
void CommandEditor::insertText(
    const std::string& value
) {

    deleteSelection();

    std::string sanitized;

    for (char character : value) {

        if (character != '\r') {
            sanitized += character;
        }
    }

    text.insert(
        cursorPosition,
        sanitized
    );

    cursorPosition +=
        sanitized.size();

    selectionAnchor =
        cursorPosition;
}

// Updates keyboard, mouse, clipboard, selection, and scrolling input.
void CommandEditor::update() {

    Vector2 mousePosition =
        GetMousePosition();

    bool mouseInside =
        CheckCollisionPointRec(
            mousePosition,
            bounds
        );

    bool shiftDown =
        IsKeyDown(KEY_LEFT_SHIFT) ||
        IsKeyDown(KEY_RIGHT_SHIFT);

    // Starts a new mouse selection.
    if (
        IsMouseButtonPressed(
            MOUSE_BUTTON_LEFT
        )
    ) {

        if (mouseInside) {

            active = true;

            cursorPosition =
                getPositionFromMouse(
                    mousePosition
                );

            selectionAnchor =
                cursorPosition;

            mouseSelecting = true;

            ensureCursorVisible();
        }
        else {

            active = false;
            mouseSelecting = false;
        }
    }

    // Extends the selection while dragging the mouse.
    if (
        mouseSelecting &&
        IsMouseButtonDown(
            MOUSE_BUTTON_LEFT
        )
    ) {

        cursorPosition =
            getPositionFromMouse(
                mousePosition
            );

        ensureCursorVisible();
    }

    if (
        IsMouseButtonReleased(
            MOUSE_BUTTON_LEFT
        )
    ) {

        mouseSelecting = false;
    }

    // Handles manual scrolling while the mouse is over the editor.
    if (mouseInside) {

        float wheel =
            GetMouseWheelMove();

        if (wheel != 0.0f) {

            if (shiftDown) {

                scrollX -=
                    wheel * 40.0f;

                float visibleWidth =
                    bounds.width -
                    2.0f * padding;

                float maxScrollX =
                    std::max(
                        0.0f,
                        getMaxLineWidth() -
                        visibleWidth
                    );

                scrollX =
                    std::clamp(
                        scrollX,
                        0.0f,
                        maxScrollX
                    );
            }
            else {

                scrollY -=
                    wheel *
                    getLineHeight() *
                    3.0f;

                float visibleHeight =
                    bounds.height -
                    2.0f * padding;

                float maxScrollY =
                    std::max(
                        0.0f,
                        getLineCount() *
                            getLineHeight() -
                            visibleHeight
                    );

                scrollY =
                    std::clamp(
                        scrollY,
                        0.0f,
                        maxScrollY
                    );
            }
        }
    }

    if (!active) {
        return;
    }

    bool controlDown =
        IsKeyDown(KEY_LEFT_CONTROL) ||
        IsKeyDown(KEY_RIGHT_CONTROL);

    // Selects the complete command.
    if (
        controlDown &&
        IsKeyPressed(KEY_A)
    ) {

        selectionAnchor = 0;
        cursorPosition =
            text.size();

        ensureCursorVisible();
        return;
    }

    // Copies the selected text to the clipboard.
    if (
        controlDown &&
        IsKeyPressed(KEY_C) &&
        hasSelection()
    ) {

        std::string selectedText =
            text.substr(
                getSelectionStart(),
                getSelectionEnd() -
                getSelectionStart()
            );

        SetClipboardText(
            selectedText.c_str()
        );

        return;
    }

    // Cuts the selected text to the clipboard.
    if (
        controlDown &&
        IsKeyPressed(KEY_X) &&
        hasSelection()
    ) {

        std::string selectedText =
            text.substr(
                getSelectionStart(),
                getSelectionEnd() -
                getSelectionStart()
            );

        SetClipboardText(
            selectedText.c_str()
        );

        deleteSelection();

        ensureCursorVisible();

        return;
    }

    // Pastes clipboard text at the cursor or replaces the selection.
    if (
        controlDown &&
        IsKeyPressed(KEY_V)
    ) {

        const char* clipboardText =
            GetClipboardText();

        if (clipboardText != nullptr) {

            insertText(
                std::string(
                    clipboardText
                )
            );

            ensureCursorVisible();
        }

        return;
    }

    // Inserts printable ASCII characters.
    if (!controlDown) {

        int character =
            GetCharPressed();

        while (character > 0) {

            if (
                character >= 32 &&
                character <= 126
            ) {

                std::string value(
                    1,
                    static_cast<char>(
                        character
                    )
                );

                insertText(value);
            }

            character =
                GetCharPressed();
        }
    }

    // Inserts a new line.
    if (IsKeyPressed(KEY_ENTER)) {

        insertText("\n");
    }

    // Inserts four spaces as indentation.
    if (IsKeyPressed(KEY_TAB)) {

        insertText("    ");
    }

    // Deletes either the selection or the previous character.
    if (IsKeyPressed(KEY_BACKSPACE) || IsKeyPressedRepeat(KEY_BACKSPACE)) {

        if (hasSelection()) {

            deleteSelection();
        }
        else if (cursorPosition > 0) {

            text.erase(
                cursorPosition - 1,
                1
            );

            --cursorPosition;

            selectionAnchor =
                cursorPosition;
        }
    }

    // Deletes either the selection or the next character.
    if (IsKeyPressed(KEY_DELETE) || IsKeyPressedRepeat(KEY_DELETE)) {

        if (hasSelection()) {

            deleteSelection();
        }
        else if (
            cursorPosition <
            text.size()
        ) {

            text.erase(
                cursorPosition,
                1
            );
        }
    }

    // Moves the cursor left or collapses the current selection.
    if (IsKeyPressed(KEY_LEFT)) {

        if (
            hasSelection() &&
            !shiftDown
        ) {

            cursorPosition =
                getSelectionStart();
        }
        else if (cursorPosition > 0) {

            --cursorPosition;
        }

        if (!shiftDown) {
            selectionAnchor =
                cursorPosition;
        }
    }

    // Moves the cursor right or collapses the current selection.
    if (IsKeyPressed(KEY_RIGHT)) {

        if (
            hasSelection() &&
            !shiftDown
        ) {

            cursorPosition =
                getSelectionEnd();
        }
        else if (
            cursorPosition <
            text.size()
        ) {

            ++cursorPosition;
        }

        if (!shiftDown) {
            selectionAnchor =
                cursorPosition;
        }
    }

    // Moves the cursor to the previous line.
    if (IsKeyPressed(KEY_UP)) {

        moveCursorVertical(-1);

        if (!shiftDown) {
            selectionAnchor =
                cursorPosition;
        }
    }

    // Moves the cursor to the next line.
    if (IsKeyPressed(KEY_DOWN)) {

        moveCursorVertical(1);

        if (!shiftDown) {
            selectionAnchor =
                cursorPosition;
        }
    }

    // Moves the cursor to the beginning of the current line.
    if (IsKeyPressed(KEY_HOME)) {

        cursorPosition =
            getCurrentLineStart();

        if (!shiftDown) {
            selectionAnchor =
                cursorPosition;
        }
    }

    // Moves the cursor to the end of the current line.
    if (IsKeyPressed(KEY_END)) {

        cursorPosition =
            getCurrentLineEnd();

        if (!shiftDown) {
            selectionAnchor =
                cursorPosition;
        }
    }

    // Removes keyboard focus from the editor.
    if (IsKeyPressed(KEY_ESCAPE)) {

        active = false;
        mouseSelecting = false;
    }

    ensureCursorVisible();
}

// Draws the editor, text, selection, and cursor.
void CommandEditor::draw() const {

    DrawRectangleRec(
        bounds,
        RAYWHITE
    );

    DrawRectangleLinesEx(
        bounds,
        2.0f,
        active ? BLUE : GRAY
    );

    Rectangle contentArea = {
        bounds.x + padding,
        bounds.y + padding,
        bounds.width - 2.0f * padding,
        bounds.height - 2.0f * padding
    };

    BeginScissorMode(
        static_cast<int>(
            contentArea.x
        ),
        static_cast<int>(
            contentArea.y
        ),
        static_cast<int>(
            contentArea.width
        ),
        static_cast<int>(
            contentArea.height
        )
    );

    Font font =
        GetFontDefault();

    float lineHeight =
        getLineHeight();

    float baseX =
        bounds.x +
        padding -
        scrollX;

    float y =
        bounds.y +
        padding -
        scrollY;

    if (text.empty()) {

        DrawTextEx(
            font,
            "Paste or type a simulation command here...",
            Vector2{
                bounds.x + padding,
                y
            },
            static_cast<float>(
                fontSize
            ),
            1.0f,
            GRAY
        );
    }
    else {

        std::size_t lineStart = 0;

        while (
            lineStart <= text.size()
        ) {

            std::size_t lineEnd =
                text.find(
                    '\n',
                    lineStart
                );

            if (
                lineEnd ==
                std::string::npos
            ) {
                lineEnd =
                    text.size();
            }

            // Draws the selected portion of the current line.
            if (hasSelection()) {

                std::size_t selectionStart =
                    getSelectionStart();

                std::size_t selectionEnd =
                    getSelectionEnd();

                std::size_t overlapStart =
                    std::max(
                        lineStart,
                        selectionStart
                    );

                std::size_t overlapEnd =
                    std::min(
                        lineEnd,
                        selectionEnd
                    );

                if (
                    overlapStart <
                    overlapEnd
                ) {

                    std::string prefix =
                        text.substr(
                            lineStart,
                            overlapStart -
                            lineStart
                        );

                    std::string selected =
                        text.substr(
                            overlapStart,
                            overlapEnd -
                            overlapStart
                        );

                    float prefixWidth =
                        MeasureTextEx(
                            font,
                            prefix.c_str(),
                            static_cast<float>(
                                fontSize
                            ),
                            1.0f
                        ).x;

                    float selectedWidth =
                        MeasureTextEx(
                            font,
                            selected.c_str(),
                            static_cast<float>(
                                fontSize
                            ),
                            1.0f
                        ).x;

                    DrawRectangle(
                        static_cast<int>(
                            baseX +
                            prefixWidth
                        ),
                        static_cast<int>(y),
                        static_cast<int>(
                            selectedWidth
                        ),
                        static_cast<int>(
                            lineHeight
                        ),
                        Fade(
                            BLUE,
                            0.25f
                        )
                    );
                }
            }

            std::string line =
                text.substr(
                    lineStart,
                    lineEnd -
                    lineStart
                );

            DrawTextEx(
                font,
                line.c_str(),
                Vector2{
                    baseX,
                    y
                },
                static_cast<float>(
                    fontSize
                ),
                1.0f,
                BLACK
            );

            y += lineHeight;

            if (
                lineEnd ==
                text.size()
            ) {
                break;
            }

            lineStart =
                lineEnd + 1;
        }
    }

    // Draws the blinking text cursor.
    if (
        active &&
        std::fmod(
            GetTime(),
            1.0
        ) < 0.5
    ) {

        std::size_t lineStart =
            getCurrentLineStart();

        std::string beforeCursor =
            text.substr(
                lineStart,
                cursorPosition -
                lineStart
            );

        float cursorX =
            baseX +
            MeasureTextEx(
                font,
                beforeCursor.c_str(),
                static_cast<float>(
                    fontSize
                ),
                1.0f
            ).x;

        float cursorY =
            bounds.y +
            padding +
            getCursorLine() *
                lineHeight -
            scrollY;

        DrawLine(
            static_cast<int>(
                cursorX
            ),
            static_cast<int>(
                cursorY
            ),
            static_cast<int>(
                cursorX
            ),
            static_cast<int>(
                cursorY +
                fontSize
            ),
            BLACK
        );
    }

    EndScissorMode();
}

// Returns the complete command stored in the editor.
const std::string&
CommandEditor::getText() const {
    return text;
}

// Replaces the complete command text.
void CommandEditor::setText(
    const std::string& value
) {

    text = value;

    cursorPosition =
        text.size();

    selectionAnchor =
        cursorPosition;

    scrollX = 0.0f;
    scrollY = 0.0f;

    ensureCursorVisible();
}

// Removes all text from the editor.
void CommandEditor::clear() {

    text.clear();

    cursorPosition = 0;
    selectionAnchor = 0;

    scrollX = 0.0f;
    scrollY = 0.0f;
}

// Returns whether the editor currently has keyboard focus.
bool CommandEditor::isActive() const {
    return active;
}
