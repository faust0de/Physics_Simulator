#pragma once

#include <raylib.h>

#include <cstddef>
#include <string>

// Provides an editable multi-line text area for simulation commands.
class CommandEditor {
private:
    // Defines the editor position and size on the screen.
    Rectangle bounds;

    // Stores the current command text.
    std::string text;

    // Stores the active cursor position inside the text.
    std::size_t cursorPosition;

    // Stores the fixed endpoint used to define a text selection.
    std::size_t selectionAnchor;

    // Indicates whether the editor currently has keyboard focus.
    bool active;

    // Indicates whether the user is selecting text with the mouse.
    bool mouseSelecting;

    // Stores the horizontal text scroll offset in pixels.
    float scrollX;

    // Stores the vertical text scroll offset in pixels.
    float scrollY;

    // Defines the editor font size.
    int fontSize;

    // Defines the spacing between text and editor borders.
    float padding;

    // Returns the vertical distance between consecutive text lines.
    float getLineHeight() const;

    // Returns the number of logical lines in the current text.
    int getLineCount() const;

    // Returns the largest line width in pixels.
    float getMaxLineWidth() const;

    // Returns the index where the line containing the cursor begins.
    std::size_t getCurrentLineStart() const;

    // Returns the index where the line containing the cursor ends.
    std::size_t getCurrentLineEnd() const;

    // Returns the zero-based line number containing the cursor.
    int getCursorLine() const;

    // Returns whether a text range is currently selected.
    bool hasSelection() const;

    // Returns the first index of the current selection.
    std::size_t getSelectionStart() const;

    // Returns the index immediately after the current selection.
    std::size_t getSelectionEnd() const;

    // Deletes the currently selected text.
    void deleteSelection();

    // Converts a mouse position into the nearest text index.
    std::size_t getPositionFromMouse(
        const Vector2& mousePosition
    ) const;

    // Moves the cursor one line upward or downward.
    void moveCursorVertical(int direction);

    // Keeps the cursor visible by adjusting horizontal and vertical scroll.
    void ensureCursorVisible();

    // Inserts text at the cursor, replacing any active selection.
    void insertText(const std::string& value);

public:
    // Creates a command editor inside the specified screen rectangle.
    CommandEditor(
        Rectangle editorBounds,
        int editorFontSize = 18
    );

    // Updates keyboard, mouse, clipboard, selection, and scrolling input.
    void update();

    // Draws the editor, text, selection, and cursor.
    void draw() const;

    // Returns the complete command stored in the editor.
    const std::string& getText() const;

    // Replaces the complete command text.
    void setText(const std::string& value);

    // Removes all text from the editor.
    void clear();

    // Returns whether the editor currently has keyboard focus.
    bool isActive() const;
};