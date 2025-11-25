# Contributing to CV Arm

Thank you for your interest in contributing to CV Arm! This document provides guidelines for contributing to the project.

## Getting Started

1. Fork the repository
2. Clone your fork:
   ```bash
   git clone https://github.com/YOUR_USERNAME/cv_arm.git
   cd cv_arm
   ```
3. Create a new branch for your feature/fix:
   ```bash
   git checkout -b feature/your-feature-name
   ```

## Development Setup

1. Install all dependencies as described in README.md
2. Build the package:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select cv_arm
   source install/setup.bash
   ```

## Coding Standards

### Python Code Style
- Follow [PEP 8](https://pep8.org/) style guide
- Use meaningful variable and function names
- Add docstrings to all functions and classes
- Maximum line length: 100 characters

### ROS2 Conventions
- Follow [ROS2 naming conventions](https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html)
- Use snake_case for node names, topics, and parameters
- Use CamelCase for class names

### Code Quality
Run tests before submitting:
```bash
# Run Python linters
cd ~/ros2_ws/src/cv_arm
python3 -m flake8 cv_arm/
python3 -m pytest test/

# Build and test package
cd ~/ros2_ws
colcon build --packages-select cv_arm
colcon test --packages-select cv_arm
```

## Making Changes

### Commit Messages
- Use clear, descriptive commit messages
- Start with a verb in present tense (e.g., "Add", "Fix", "Update")
- Keep the first line under 50 characters
- Add detailed description if needed

Example:
```
Add support for multi-joint control

- Extend gesture detector to control shoulder joint
- Update URDF model with new joints
- Add configuration for additional controllers
```

### Pull Request Process

1. Update documentation if you've changed functionality
2. Add tests for new features
3. Ensure all tests pass
4. Update CHANGELOG.md (if exists) with your changes
5. Submit a pull request with a clear description of:
   - What problem it solves
   - How it solves it
   - Any breaking changes
   - Screenshots/videos if applicable

## Reporting Bugs

When reporting bugs, please include:
- Your OS version and ROS2 distribution
- Python version
- Steps to reproduce the issue
- Expected vs actual behavior
- Relevant log output
- Screenshots/videos if applicable

## Feature Requests

We welcome feature requests! Please:
- Check if the feature has already been requested
- Clearly describe the feature and use case
- Explain why it would be useful to most users

## Questions?

If you have questions, please:
- Check existing documentation (README, ARCHITECTURE.md, QUICKSTART.md)
- Search existing issues
- Open a new issue with the "question" label

## Code of Conduct

- Be respectful and inclusive
- Provide constructive feedback
- Focus on what is best for the community
- Show empathy towards other community members

## License

By contributing, you agree that your contributions will be licensed under the Apache License 2.0.
