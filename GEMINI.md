# SEL0359 - Controle Digital (Slides)

This project contains the educational slides for the course **SEL0359 - Controle Digital** at the Escola de Engenharia de São Carlos (EESC-USP). The slides are built using the [Reveal.js](https://revealjs.com/) framework and rendered in HTML.

## Project Overview

- **Subject:** Digital Control (Controle Digital)
- **Framework:** Reveal.js
- **Mathematics:** KaTeX (configured for LaTeX-style equations)
- **Organization:** Slides are organized by year (e.g., `2024/`, `2025/`) and topic.

## Directory Structure

- `2024/`, `2025/`: Contain the lecture slides as individual HTML files.
- `images/`: Contains diagrams, plots, and photos used in the slides.
- `videos/`: Contains animations and demonstration videos.
- `dist/`, `plugin/`: Reveal.js core library and plugins (Markdown, Notes, KaTeX).
- `init.js`: Central initialization script for Reveal.js, including KaTeX macro definitions (e.g., `\R`, `\trp`, `\diag`).
- `style.css`: Custom CSS styles for the presentation layout and specific course elements (e.g., `.lightup` blocks).
- `pdf/`: Directory for PDF versions of the slides.

## Key Files

- `init.js`: Configures the presentation behavior (transitions, slide size, math delimiters).
- `style.css`: Defines the visual theme, including USP/EESC logos and custom content containers.
- `2025/aula1.html`: An example of a complete lecture introduction and course overview.

## Usage

### Viewing Slides
To view the slides, open any HTML file in a modern web browser. For example:
- `2025/aula1.html`

### Development
- **Adding Slides:** Create a new HTML file in the current year's directory, referencing `../dist/`, `../plugin/`, `../style.css`, and `../init.js`.
- **Equations:** Use `$` for inline math and `$$` for block math. Custom macros are defined in `init.js`.
- **Custom Classes:** 
  - `.lightup`: Highlighted block for key concepts or formulas.
  - `.slide-title`: Standardized container for slide headers.
  - `.slide-content`: Container for main slide text/media.

### Dependencies
The project relies on local copies of Reveal.js located in `dist/` and `plugin/`. It does not require a build step unless you are modifying the Reveal.js source itself.
