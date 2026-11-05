# RUDRA PROPRIETARY AND RESTRICTED EDUCATIONAL SOFTWARE LICENSE

## Version 1.0 — November 2026


**License Identifier:** RPREL-1.0  
**Project:** RUDRA Programming Language  
**Compiler:** `rudrac`  
**Source Extension:** `.rudra`  
**Copyright:** © 2026 Rudra Project Contributors. All Rights Reserved.  
**License Classification:** Proprietary / Source-Available / Restricted Educational Use

---

# PREAMBLE

The RUDRA Programming Language (hereinafter referred to as "RUDRA") is an independently developed domain-specific programming language and associated compilation ecosystem created for the development, coordination, configuration, and control of embedded slave nodes within the RUDRA FireBot robotic architecture.

RUDRA is designed around deterministic execution, constrained embedded environments, declared output states, safety interlocks, bounded execution, hardware abstraction, protocol generation, and verification-oriented development.

The RUDRA ecosystem includes, but is not limited to:

- The RUDRA programming language and its syntax.
- The `.rudra` source-code format.
- The `rudrac` compiler and associated command-line interfaces.
- Lexical analysis, parsing, semantic analysis, and type-checking components.
- Intermediate representation and compiler transformation systems.
- Code-generation backends.
- Generated Arduino and ESP32 C++ source code.
- Master-node communication interfaces and protocol generators.
- Python communication clients.
- Simulation and virtual-time execution systems.
- Static verification and interlock-analysis systems.
- Resource-cost estimation and execution-budget analysis.
- Hardware abstraction definitions.
- Standard language constructs and built-in operations.
- Documentation, examples, test suites, and associated development materials.

This License establishes the conditions under which individuals may access, study, execute, and interact with the RUDRA software.

By accessing, downloading, installing, copying, executing, modifying, distributing, or otherwise interacting with any portion of the RUDRA Software, the recipient agrees to the applicable terms of this License.

If the recipient does not agree to these terms, they must discontinue use of the RUDRA Software.

# ARTICLE I — DEFINITIONS

### 1.1. Licensed Software

"Licensed Software" means the complete RUDRA ecosystem, including source code, executable components, compiler implementations, generated tooling, documentation, examples, test infrastructure, language specifications, and associated materials.

### 1.2. Original Work

"Original Work" means the original implementation of RUDRA, including its language design, compiler architecture, intermediate representation, verification methodology, execution model, syntax, and associated engineering decisions.

### 1.3. Contributor

"Contributor" means an individual who has authored, implemented, designed, or submitted an accepted contribution to the RUDRA project.

### 1.4. Recipient

"Recipient" means any person or legal entity that accesses or obtains the Licensed Software.

### 1.5. Educational Use

"Educational Use" means personal learning, academic demonstrations, classroom exercises, non-commercial experimentation, and university or college project evaluation.

### 1.6. Commercial Use

"Commercial Use" includes any use intended to generate revenue, financial advantage, licensing fees, subscription income, commercial services, or other economic benefits.

### 1.7. Derivative Work

"Derivative Work" means any software, language, compiler, framework, implementation, documentation, or other material substantially based upon, incorporating, translating, adapting, or modifying the Licensed Software.

### 1.8. RUDRA Ecosystem

"RUDRA Ecosystem" means all software and technical components developed as part of or specifically for the RUDRA language and its associated FireBot architecture.

### 1.9. Generated Output

"Generated Output" means source code, intermediate files, headers, protocol definitions, executables, or other artifacts produced by the RUDRA compiler.

### 1.10. Authorized Permission

"Authorized Permission" means explicit written permission granted by the copyright holders or their authorized representatives.

# ARTICLE II — OWNERSHIP AND INTELLECTUAL PROPERTY

### 2.1. Ownership

All rights, title, and interest in the original Licensed Software remain with their respective copyright holders and contributors, subject to applicable intellectual property law.

### 2.2. Reserved Rights

All rights not expressly granted under this License are reserved.

No provision of this License shall be interpreted as transferring ownership of the RUDRA programming language, compiler architecture, source implementation, or associated intellectual property to a Recipient.

### 2.3. Language Identity

The name RUDRA, its associated project identity, and its original language branding may not be used to represent an independently developed or modified implementation as the official RUDRA project without authorization.

### 2.4. Compiler Architecture

The internal compiler architecture, intermediate representation, semantic rules, verification mechanisms, and original implementation are protected materials under this License.

### 2.5. No Implied Assignment

Access to the Licensed Software does not grant ownership, co-ownership, or an automatic intellectual property interest in the original project.

### 2.6. Third-Party Components

Third-party software incorporated into or used alongside RUDRA remains subject to its respective license.

This License does not override the independent rights or obligations applicable to such components.

# ARTICLE III — LIMITED EDUCATIONAL LICENSE

### 3.1. Grant of Permission

Subject to compliance with this License, the copyright holders grant the Recipient a limited, personal, non-exclusive, non-transferable, non-sublicensable, revocable permission to:

1. Access the RUDRA source repository.
2. Download the Licensed Software.
3. Read and examine the source code.
4. Install and execute the compiler.
5. Write and execute personal `.rudra` programs.
6. Perform non-commercial experiments.
7. Use the software for educational demonstrations.
8. Evaluate the compiler and its generated output.
9. Execute the provided examples and test suites.
10. Reference the project in academic discussions with appropriate attribution.

### 3.2. Scope

The permission granted under this Article is strictly limited to individual educational and personal experimentation.

### 3.3. No Automatic Redistribution Rights

The educational permission does not independently grant permission to republish, mirror, redistribute, sublicense, or commercially package the Licensed Software.

### 3.4. No Ownership Rights

The Recipient receives a limited right of use, not ownership of the Licensed Software.

# ARTICLE IV — STRICT RESTRICTIONS

Unless expressly authorized in writing, the following activities are prohibited.

### 4.1. Unauthorized Redistribution

The Recipient shall not:

- Republish the complete source repository.
- Upload copies of the compiler to another repository.
- Distribute modified or unmodified compiler packages.
- Create unofficial downloadable distributions.
- Mirror the repository as an independently maintained distribution.
- Include the complete Licensed Software in another software package.
- Redistribute proprietary portions of the project through package registries.

### 4.2. Unauthorized Commercialization

The Recipient shall not:

- Sell the RUDRA compiler.
- Charge licensing fees for the Licensed Software.
- Offer paid access to the original compiler.
- Incorporate RUDRA into a commercial product.
- Offer commercial RUDRA compilation services.
- Package RUDRA as a paid development framework.
- Use the original project to provide paid proprietary services.
- Claim commercial licensing rights without written authorization.

### 4.3. Unauthorized Relicensing

The Recipient shall not:

- Replace this License with MIT, Apache-2.0, GPL, BSD, or another license.
- Remove or alter the original copyright notices.
- Claim that RUDRA is released under an open-source license.
- Issue sublicenses over the original Licensed Software.
- Represent the original project as public domain.

### 4.4. Unauthorized Rebranding

The Recipient shall not distribute a modified version while claiming that it is the official, original, or officially maintained RUDRA compiler.

### 4.5. Unauthorized Extraction

The Recipient shall not extract substantial portions of the compiler implementation, verification engine, or language infrastructure for incorporation into another independently distributed product without authorization.

### 4.6. Attribution

Where references to RUDRA are permitted, the original project and its contributors must receive appropriate attribution.

Attribution must not falsely imply endorsement, sponsorship, certification, or official association.

# ARTICLE V — MODIFICATION AND DERIVATIVE IMPLEMENTATIONS

### 5.1. Modification

Modification of the original Licensed Software is not included in the general educational permission.

A Recipient may independently experiment with changes in a private educational environment, but may not publicly distribute those modifications without written authorization.

### 5.2. Compiler Forks

Publicly distributing an independent fork of `rudrac`, including a modified compiler, is prohibited without authorization.

### 5.3. Language Forks

Creating and independently distributing a modified RUDRA language implementation based substantially on the original implementation is prohibited without authorization.

### 5.4. Derivative Languages

The Recipient shall not claim that a derivative language is the official RUDRA language.

### 5.5. Architecture Replication

This Article does not prohibit independent engineering research or independently developed implementations that do not unlawfully incorporate protected expression, confidential information, or other protected material.

### 5.6. Academic Analysis

Academic analysis, criticism, commentary, and appropriately limited quotations are permitted where allowed by applicable law.

### 5.7. Contributions

Submitting a contribution to the project does not automatically transfer ownership of the contributor's independent intellectual property.

Contributions are subject to any separately agreed contribution terms and applicable law.

# ARTICLE VI — GENERATED CODE AND COMPILATION OUTPUT

### 6.1. Generated Output

RUDRA may generate Arduino C++, ESP32-compatible source files, communication headers, Python clients, and protocol documentation.

### 6.2. Independent User Programs

The Recipient retains ownership of original `.rudra` programs they independently create, subject to applicable third-party rights.

### 6.3. Generated Code

This License does not automatically claim ownership of every generated output.

Generated output may contain portions of compiler templates, runtime support code, or third-party components subject to their respective rights and licenses.

### 6.4. Generated Interfaces

The Recipient may use generated interfaces for personal educational projects, subject to the applicable restrictions governing the compiler, runtime components, and third-party dependencies.

### 6.5. No Automatic Commercial Grant

Possession of generated output does not automatically grant commercial distribution rights over embedded compiler-owned components or separately licensed materials.

# ARTICLE VII — HARDWARE, ROBOTICS, AND SAFETY LIMITATIONS

### 7.1. Intended Environment

RUDRA is developed primarily for constrained embedded slave-node applications within the RUDRA FireBot architecture.

Its intended environments include Arduino Nano, Arduino Uno, ESP32, ESP32-S3, and compatible embedded platforms.

### 7.2. No Safety Certification

RUDRA is not a certified industrial safety controller, medical device controller, aviation control system, or certified life-critical software platform.

### 7.3. No Guarantee of Physical Safety

The existence of declared safe states, software interlocks, static analysis, simulation, verification, or bounded execution estimates does not establish that a physical machine is safe.

### 7.4. Independent Hardware Protection

Recipients are responsible for implementing appropriate independent hardware protections, including where applicable:

- Emergency stop circuits.
- Thermal fuses.
- Physical power isolation.
- Overcurrent protection.
- Mechanical limit switches.
- Independent relay protection.
- Battery protection circuits.
- Hardware watchdogs.
- Appropriate electrical isolation.
- Physical safeguards against unintended movement.

### 7.5. Human Safety

The Licensed Software shall not be represented as providing guaranteed human-rescue capability or guaranteed protection against injury, fire, equipment failure, or property damage.

### 7.6. Testing Responsibility

Recipients are responsible for evaluating generated code, electrical interfaces, hardware compatibility, timing assumptions, and operating conditions before deploying software to physical equipment.

### 7.7. FireBot Applications

The association of RUDRA with the FireBot project does not constitute a certification or guarantee that any resulting robot can safely enter hazardous environments, extinguish fires, or rescue individuals.

### 7.8. Physical Deployment

Any physical deployment is undertaken at the Recipient's own responsibility and must comply with applicable safety requirements and regulations.

# ARTICLE VIII — NO WARRANTY

### 8.1. As-Is Provision

THE LICENSED SOFTWARE IS PROVIDED "AS IS" AND "AS AVAILABLE", WITHOUT WARRANTIES OF ANY KIND, EXPRESS OR IMPLIED, TO THE MAXIMUM EXTENT PERMITTED BY APPLICABLE LAW.

### 8.2. Excluded Warranties

To the maximum extent permitted by law, the copyright holders disclaim warranties concerning:

- Merchantability.
- Fitness for a particular purpose.
- Non-infringement.
- Accuracy.
- Reliability.
- Availability.
- Compatibility.
- Security.
- Compilation correctness.
- Hardware compatibility.
- Execution determinism.
- Resource-estimation accuracy.
- Simulation equivalence.
- Verification completeness.
- Error-free operation.

### 8.3. No Guarantee of Maintenance

The copyright holders are not obligated to provide updates, patches, technical support, security fixes, compatibility improvements, or maintenance services.

### 8.4. Experimental Software

The Recipient acknowledges that RUDRA is an evolving software and language-development project.

Its syntax, compiler behavior, supported hardware, verification mechanisms, and generated interfaces may change between versions.

# ARTICLE IX — LIMITATION OF LIABILITY

### 9.1. Exclusion of Damages

To the maximum extent permitted by applicable law, the copyright holders and contributors shall not be liable for indirect, incidental, special, exemplary, punitive, or consequential damages arising from the use or inability to use the Licensed Software.

### 9.2. Categories of Loss

This exclusion includes, without limitation:

- Equipment damage.
- Hardware failures.
- Data loss.
- Business interruption.
- Lost revenue.
- Property damage.
- Personal injury.
- System failures.
- Robot malfunction.
- Unexpected motor movement.
- Electrical faults.
- Thermal incidents.
- Communication failures.
- Incorrect generated output.
- Failure of software interlocks.

### 9.3. Independent Responsibility

The Recipient assumes responsibility for the integration, testing, deployment, and operation of any system incorporating the Licensed Software.

### 9.4. Mandatory Legal Limitations

Nothing in this License excludes liability where such exclusion is prohibited by applicable law.

# ARTICLE X — ATTRIBUTION AND PROJECT IDENTITY

### 10.1. Original Attribution

Where permitted references are made to RUDRA, the following identification should be retained:

**RUDRA Programming Language**  
A domain-specific embedded programming language developed for the RUDRA FireBot project.

**Compiler:** `rudrac`  
**Source Format:** `.rudra`  
**License:** RPREL-1.0

### 10.2. No False Endorsement

Recipients shall not imply that their independent projects, products, services, or implementations are officially approved by the RUDRA project.

### 10.3. Trademark Rights

No trademark, service mark, logo, or branding rights are granted by this License.

### 10.4. Project Recognition

The original project identity must not be deliberately removed from permitted public references in a manner that misrepresents the origin of the software.

# ARTICLE XI — SECURITY AND RESPONSIBLE DISCLOSURE

### 11.1. Security Research

Non-commercial security analysis of the Licensed Software is permitted within the scope of applicable law.

### 11.2. Responsible Disclosure

Recipients discovering vulnerabilities that materially affect the compiler, generated software, or associated communication infrastructure are encouraged to report them responsibly to the project maintainers.

### 11.3. Unauthorized Exploitation

This License does not grant permission to compromise third-party systems, bypass access controls, interfere with equipment, or use RUDRA-related software for unlawful activity.

### 11.4. No Security Guarantee

The presence of verification and safety-oriented language constructs does not guarantee that the complete software or resulting hardware system is free from vulnerabilities.

# ARTICLE XII — LICENSE TERMINATION

### 12.1. Automatic Termination

Permissions granted under this License terminate automatically if the Recipient materially violates its terms, subject to applicable law.

### 12.2. Cessation of Unauthorized Use

Upon termination, the Recipient must discontinue activities that depended on the terminated permissions.

### 12.3. Existing Independent Rights

Termination does not automatically revoke rights independently held in original user-authored programs or third-party components.

### 12.4. Survival

The provisions concerning ownership, intellectual property, liability, disclaimers, attribution, and other provisions intended by their nature to survive shall remain applicable after termination.

### 12.5. Restoration

Permissions may be restored through explicit written authorization from the copyright holders.

# ARTICLE XIII — VERSIONING AND AMENDMENTS

### 13.1. License Version

This document constitutes RPREL-1.0.

### 13.2. Amendments

The copyright holders may publish revised versions of this License for future releases.

### 13.3. No Automatic Retroactive Expansion

Publication of a newer version does not automatically expand permissions granted under earlier versions.

### 13.4. Version Identification

Each release may identify the applicable license version in its repository metadata and accompanying documentation.

### 13.5. Continued Use

Continued use of the Licensed Software remains subject to the applicable license terms.

# ARTICLE XIV — GOVERNING LAW AND JURISDICTION

### 14.1. Applicable Law

This License shall be interpreted in accordance with the laws of India, subject to applicable mandatory legal provisions.

### 14.2. Jurisdiction

Subject to applicable law, disputes arising from this License shall be submitted to the competent courts having jurisdiction in Maharashtra, India.

### 14.3. Statutory Rights

Nothing in this License shall be interpreted as eliminating statutory rights or protections that cannot lawfully be waived.

# ARTICLE XV — GENERAL PROVISIONS

### 15.1. Entire Agreement

This License constitutes the licensing terms applicable to the Licensed Software unless superseded by a separate written agreement.

### 15.2. Severability

If any provision is held invalid or unenforceable, the remaining provisions shall continue to operate to the maximum extent permitted by law.

### 15.3. No Waiver

Failure to enforce any provision shall not constitute a permanent waiver of that provision.

### 15.4. No Partnership

Nothing in this License creates a partnership, employment relationship, agency, joint venture, or other organizational relationship between the Recipient and the copyright holders.

### 15.5. No Assignment

The Recipient may not transfer or assign the permissions granted under this License without written authorization.

### 15.6. Interpretation

Headings are included for organizational convenience and do not independently alter the meaning of the provisions.

### 15.7. Electronic Acceptance

Accessing or using the Licensed Software constitutes acceptance of these terms to the extent permitted by applicable law.

---

# ARTICLE XVI — SPECIAL PROJECT PROVISIONS

### 16.1. Dedicated Language Identity

RUDRA is a dedicated programming language developed for a specific embedded robotics ecosystem.

Its original design identity, compiler implementation, and associated infrastructure are recognized as part of the RUDRA project.

### 16.2. Dedicated Compiler Identity

The `rudrac` compiler is the reference compiler implementation of the RUDRA programming language.

Unofficial implementations shall not be represented as the official compiler.

### 16.3. Safety-Oriented Design

The language's safe-state declarations, interlock rules, bounded execution model, verification mechanisms, and related constructs constitute important aspects of its original design.

### 16.4. Ecosystem Integrity

The Recipient shall not deliberately misrepresent the behavior, capabilities, verification results, or hardware guarantees of the original RUDRA implementation.

### 16.5. Independent Development

Nothing in this License prevents independent development of programming languages, embedded systems, robotics software, compilers, simulators, or verification tools, provided such development respects applicable intellectual property rights.

### 16.6. Academic Recognition

The use of RUDRA in an academic project, demonstration, technical presentation, research discussion, or educational exercise is permitted within the limits of this License.

### 16.7. Official Project Status

Only implementations expressly designated by the project maintainers may claim official RUDRA project status.

---

# FINAL LICENSE DECLARATION

The RUDRA Programming Language is an independently developed software engineering project created for embedded robotics and the RUDRA FireBot ecosystem.

This License preserves the original project's intellectual property, restricts unauthorized redistribution and commercialization, protects project identity, and grants limited educational access to individuals interested in studying and experimenting with the language.

The rights granted herein are limited and conditional.

**ALL RIGHTS NOT EXPRESSLY GRANTED ARE RESERVED.**

---

**RUDRA PROGRAMMING LANGUAGE**  
**RPREL-1.0**

© 2026 Rudra Project Contributors. All Rights Reserved.
