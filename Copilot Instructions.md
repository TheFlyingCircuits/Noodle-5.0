# GitHub Copilot / Cursor Instructions for FRC Development

You are assisting with a FIRST Robotics Competition (FRC) project using WPILib. You have access to an MCP server that provides documentation search across WPILib and vendor libraries (REV, CTRE, Redux, etc.).

IMPORTANT: Before answering any question about FRC programming, motor controllers, sensors, WPILib, or vendor APIs, you SHOULD run the MCP documentation search/fetch tools to verify your answer and provide citations.

## Documentation-First Policy with Smart Fallback

Use MCP tools to search documentation, but apply judgment about when to trust results vs. your own knowledge:

**When to trust search results:**
- Results have high confidence scores (> 0.5)
- Multiple relevant results from the same vendor
- Results directly mention the exact API/hardware asked about

**When to use your knowledge instead:**
- Search returns no results or very low confidence
- Results seem irrelevant to the actual question
- You have strong knowledge about a common FRC topic

Required workflow for FRC questions:

- **Step 1 — Format Query:** Convert the user's question into effective search keywords (see Query Formatting below)
- **Step 2 — Search:** Call `mcp_wpilib_search_frc_docs(query=..., vendors=[...])`. Check the `confidence` scores and `suggestions` in results.
- **Step 3 — Evaluate Results:** If confidence is low or results seem off, try alternative queries or use your knowledge with appropriate caveats.
- **Step 4 — Fetch:** For high-confidence results, call `mcp_wpilib_fetch_frc_doc_page(url=...)` to get full content.
- **Step 5 — Answer with citations:** Include documentation URLs when available. If using your knowledge, note that documentation wasn't available.

## Query Formatting Guidelines

**CRITICAL:** Format queries as keyword searches, not natural language questions.

| User Question | BAD Query | GOOD Query |
|---------------|-----------|------------|
| "How do I configure a SparkMax?" | "how do I configure a sparkmax" | "SparkMax configuration" |
| "What's the best way to set up PID?" | "what is the best way to set up pid" | "PID controller setup" |
| "Can you show me CAN bus wiring?" | "can you show me can bus wiring" | "CAN bus wiring" |

**Query formatting rules:**
1. Remove question words (how, what, why, can, etc.)
2. Use the exact product names: `SparkMax`, `TalonFX`, `CANcoder`, `NEO`
3. Keep FRC acronyms intact: `CAN`, `PID`, `PWM`, `DIO`
4. Use 2-4 keywords, not full sentences
5. Include the action: "configure", "setup", "wiring", "code example"

**Examples of good queries:**
```
"SparkMax brushless configuration"
"TalonFX PID tuning"
"CAN bus troubleshooting"
"command-based subsystem example"
"swerve drive odometry"
"NEO current limit"
```

## Understanding Search Results

The search returns results with these fields:
- `confidence`: 0.0-1.0 score (higher = more relevant)
- `suggestions`: Alternative queries if results are weak

**Interpreting confidence:**
- `> 0.7`: High confidence - trust these results
- `0.4 - 0.7`: Medium confidence - results may be relevant
- `< 0.4`: Low confidence - consider rephrasing or using your knowledge

If you see suggestions like `'sparkmax' → 'spark max'`, try the suggested term.

## Tool Usage Patterns

**For general questions (MCP):**
```
mcp_wpilib_search_frc_docs(query="how to configure PID", vendors=["all"])
```

**For vendor-specific questions (MCP):**
```
mcp_wpilib_search_frc_docs(query="SparkMax current limits", vendors=["rev"])
mcp_wpilib_search_frc_docs(query="TalonFX motion magic", vendors=["ctre"])
```

**For comparisons (MCP):**
```
mcp_wpilib_search_frc_docs(query="brushless motor setup", vendors=["rev"], max_results=5)
mcp_wpilib_search_frc_docs(query="brushless motor setup", vendors=["ctre"], max_results=5)
```

**After finding relevant pages (MCP):**
```
mcp_wpilib_fetch_frc_doc_page(url="https://docs.revrobotics.com/...")
```

## Automatic Context Detection

The search tool **automatically detects** the project's programming language and vendor libraries by scanning:
- File extensions (.java, .py, .cpp)
- Build files (build.gradle, pyproject.toml, CMakeLists.txt)
- Vendordeps folder (vendordeps/*.json)
- Import statements in source code

**This means you usually don't need to specify `language` or `vendors`!**

```
# Let auto-detection handle it (recommended)
mcp_wpilib_search_frc_docs(query="SparkMax configuration")

# Or disable auto-detection if needed
mcp_wpilib_search_frc_docs(query="SparkMax configuration", auto_detect=false, vendors=["rev"], language="Java")
```

**To see what was detected:**
```
mcp_wpilib_detect_project_context()
```

This returns the detected language, vendors, and confidence level. Use this at the start of a session to understand the project's stack.

## Language and Version Awareness

- The language is **auto-detected** from the project. Only ask the student if detection fails.
- Default to the current season (2025) unless specified otherwise
- Override auto-detection when needed:
```
search_frc_docs(query="command based", language="Python", version="2025", auto_detect=false)
```

## Code Style for FRC

When writing code for FRC projects:

- Follow WPILib conventions (Command-based or TimedRobot patterns)
- **Do not instantiate vendor SDK classes (SparkMax, TalonFX) directly in robot code.** This team uses in-house wrapper classes instead — see "Motor Controller Conventions" below before writing any motor code.
- Include necessary imports
- Add comments explaining the "why" for students learning
- Handle units properly (RPM vs rotations, degrees vs radians)

## Motor Controller Conventions

This codebase does NOT interact with vendor motor controller classes
(SparkMax, TalonFX) directly. Instead, we use in-house wrapper classes:

- `Neo` wraps REV SparkMax (for NEO motors) — defined in: `<PATH_TO_NEO_CLASS>`
- `Kraken` wraps CTRE TalonFX (for Kraken motors) — defined in: `<PATH_TO_KRAKEN_CLASS>`

**Before writing or suggesting any motor control code, open and read the
files above** to see the wrapper classes' real method signatures — do
not assume they expose the same API as the raw vendor SDK classes, and
do not suggest instantiating `SparkMax` or `TalonFX` directly anywhere
in team code. If either file has moved, search the repo for `class Neo`
or `class Kraken` as a fallback.

The vendor names below (REV, CTRE, etc.) are still the correct terms to
use when searching FRC **documentation** — the wrapper classes only change
how motors are instantiated and called in our own code, not what the
underlying hardware or vendor docs are called.

## Common Vendor Mappings

Use this table only to pick the right `vendors` filter when searching
**documentation**. It does not describe how to write motor code — see
"Motor Controller Conventions" above for that.

| Hardware | Vendor | Search with |
|----------|--------|-------------|
| SparkMax, SparkFlex, NEO, NEO 550 | REV | `vendors=["rev"]` |
| TalonFX, Falcon 500, Kraken, CANcoder, Pigeon | CTRE | `vendors=["ctre"]` |
| Canandcoder, Canandmag | Redux | `vendors=["redux"]` |
| NavX | WPILib/Studica | `vendors=["wpilib"]` |
| Limelight | WPILib | `vendors=["wpilib"]` |
| PhotonVision | PhotonVision | `vendors=["photonvision"]` |

## When Docs Don't Have the Answer

If search results are empty, low confidence, or seem irrelevant:

1. **Rephrase the query** using the formatting guidelines above
2. **Try synonyms:** The search expands common terms automatically, but you can try:
   - "motor controller" ↔ "SparkMax" / "TalonFX"
   - "encoder" ↔ "CANcoder" / "through bore"
   - "gyro" ↔ "Pigeon" / "NavX"
3. **Broaden the vendor:** Try `vendors=["all"]`
4. **Check suggestions:** The search may suggest alternative spellings
5. **Use your knowledge:** If search consistently fails, you likely have accurate knowledge about the topic. Answer with a note like: "I couldn't find this in the current documentation index, but based on my knowledge..."

**Don't apologize excessively** for missing docs. The documentation index doesn't cover everything - it's a supplement, not a replacement for your knowledge.

## Example Interaction

**Student:** "How do I set up a SparkMax for a NEO brushless motor?"

**You should:**
1. `mcp_wpilib_search_frc_docs(query="SparkMax NEO brushless setup", vendors=["rev"])`
2. Review results, pick most relevant URL
3. `mcp_wpilib_fetch_frc_doc_page(url="...")` to get full content
4. Open the team's `Neo` wrapper class (see "Motor Controller Conventions") to check its actual constructor and methods
5. Write code using the `Neo` wrapper — not `SparkMax` directly — based on current documentation and the wrapper's real API
6. Tell the student: "Based on the REV documentation at [url] and our `Neo` wrapper class, here's how to set it up..."