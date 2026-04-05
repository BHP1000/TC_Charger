# Contributing to TC_Charger

Thank you for your interest in contributing! This project is open source under the
**CC BY-NC-SA 4.0** license with specific terms for commercial use.

## Rules

1. **All contributions are open source.** By submitting a pull request, you agree
   your work is licensed under CC BY-NC-SA 4.0. It will remain freely available for
   non-commercial use.

2. **No commercial use without a license.** You may not use this project or
   derivatives for commercial purposes without a separate agreement with the project
   owner. If you want to commercialize, reach out first.

3. **Share alike.** If you build on this project, your additions must be shared
   under the same license.

4. **Private elements by agreement only.** If you have proprietary additions you
   want to keep private as part of a commercial license, this must be explicitly
   agreed upon in writing with the project owner.

## How to Contribute

1. Fork the repo
2. Create a feature branch: `git checkout -b feature/my-feature`
3. Make your changes
4. Test on real hardware if possible
5. Submit a pull request with:
   - What you changed and why
   - What hardware you tested on (if applicable)
   - Any safety considerations

## What We're Looking For

- Support for additional charger models and CAN protocols
- Battery pack configurations beyond 20S
- Temperature sensor integrations
- Mobile app development (Android/iOS)
- Improved charging algorithms (taper detection, cell balancing integration)
- Safety improvements
- Documentation and wiring guides

## Safety

This project controls high-voltage battery charging. If your contribution affects
charging behavior, voltage limits, or current control:

- Document the safety implications clearly
- Include appropriate limit checks in code
- Test thoroughly before submitting
