# Releasing CotekCLion Firmware

This project uses **semantic versioning**: MAJOR.MINOR.PATCH (e.g. 0.3.2).
Tags trigger GitHub Actions to build firmware artifacts (.bin/.hex) automatically.

---

## 0) Preconditions

- CI green on main (last commit passes).
- Working tree clean: git status shows nothing to commit.
- You have push rights to the repo.

---

## 1) Bump the version in source

Update the firmware string in **Core/Src/main.c**:

`c
// Firmware Version
const char FW_VERSION_STR[] = "0.3.2";   // <- bump this

