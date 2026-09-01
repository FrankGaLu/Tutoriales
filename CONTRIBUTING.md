Contributing to Tutoriales

Thank you for your interest! This repository contains tutorial markdown files intended for students and hobbyists.

How to contribute

1. Fork the repository and create a feature branch from main.
2. Make small, focused edits. Keep commits atomic and include a clear message.
3. Run local checks before opening a PR:
   - Validate Python snippets: python3 -m py_compile <path-to-snippet>.py (or use the CI job that extracts snippets and compiles them).
   - Check links: npm install -g markdown-link-check && markdown-link-check README.md
4. Open a Pull Request against main and reference any relevant issues.

Guidelines

- Keep examples simple and runnable.
- When adding code snippets, prefer including minimal reproducible examples and note dependencies.
- Add images under ros2_tutorial_images/ or a new docs/images/ directory and reference them with relative paths.

License

The repository is provided under the MIT license (adjust as needed).