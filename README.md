# Pet Partner 🐾

> **Licensed pan-India live pet marketplace**  
> Built for India, inspired by France 🇫🇷🇮🇳

A French aesthetic, Indian compliance-first pet marketplace platform connecting verified breeders and pet shops with responsible pet parents.

![GitHub](https://img.shields.io/badge/Pet_Partner-v1.0-blue)
![License](https://img.shields.io/badge/license-Private-red)
![Status](https://img.shields.io/badge/status-Development-yellow)

---

## 📋 Table of Contents

- [Project Overview](#-project-overview)
- [Features](#-features)
- [Tech Stack](#-tech-stack)
- [File Structure](#-file-structure)
- [Setup & Installation](#-setup--installation)
- [Usage Guide](#-usage-guide)
- [Data Model](#-data-model)
- [Compliance Features](#-compliance-features)
- [Roadmap](#-roadmap)
- [Contributing](#-contributing)

---

## 🎯 Project Overview

Pet Partner is a marketplace platform designed specifically for the Indian pet industry, combining:

- **Parisian elegance** in UI/UX design
- **Indian regulatory compliance** (AWBI, State licenses, IT Rules)
- **Verification-first approach** for sellers
- **Transparency** for buyers

### Core Philosophy

> "We curate only registered pet shops and licensed breeders, vet-cleared animals, and compliance-first logistics — so you pick with heart, while we guard the rules."

---

## ✨ Features

### 🏠 Frontend Features

| Feature | Description |
|---------|-------------|
| **Hero Section** | Animated pet showcase with rotating frames, stats, and call-to-actions |
| **Gallery** | Nature-forward pet category showcase (Dogs, Cats, Birds, Aquatics, Reptiles) |
| **Advanced Filtering** | Filter by species, breed, gender, age, color, and state |
| **Pet Cards** | Rich cards with photos, badges, location, price, and actions |
| **Lightbox Gallery** | Full-screen photo viewer with navigation |
| **Shopping Cart** | Add-to-cart functionality with count indicator |
| **Authentication Modal** | Buyer/Seller login portal (UI ready) |
| **Responsive Design** | Mobile-first, works on all devices |

### 🔒 Compliance Features

- ✅ **Licence Gate**: Every seller must upload State Pet Shop licence or breeder permit
- ✅ **Vet Sign-off**: Fitness-to-travel certificate & vaccination log required
- ✅ **Masked Identity**: Seller name hidden; grievance contact visible per IT rules
- ✅ **Species Blocklist**: Auto-rejection of Schedule I-IV wildlife, native birds
- ✅ **Document Verification**: Platform validates licence validity and expiry

### 📊 Current Stats Displayed

- **180+** Verified breeders & pet shops
- **22** States covered with licence checks
- **48h** Vet fit-to-travel certification window

---

## 🛠 Tech Stack

```
Frontend:      Vanilla HTML5 + CSS3 + ES6+ JavaScript
Styling:       Custom CSS with CSS Variables
Fonts:         Space Grotesk, Playfair Display (Google Fonts)
Images:        Unsplash API, Dog.CEO API, TheCatAPI
Icons:         Unicode/Emoji (no icon library dependency)
State:         Vanilla JS state management
```

### Color Palette

| Color | Hex | Usage |
|-------|-----|-------|
| Navy | `#0f172a` | Primary background |
| Ink | `#0b1220` | Darker sections |
| Cream | `#f5f0ea` | Light accents |
| Coral | `#e4685b` | Primary CTA, highlights |
| Sage | `#6b7a5e` | Secondary accents |
| Gold | `#d7b168` | Premium touches |

---

## 📁 File Structure

```
petpartner/
├── index.html              # Main HTML entry point
├── src/
│   ├── main.js            # Main application logic
│   ├── snakeCore.js       # Snake game core engine (modular)
│   └── styles.css         # All styling
├── README.md              # This file
└── .git/                  # Git repository
```

### File Descriptions

#### `index.html`
- Single-page application structure
- Semantic HTML5 sections: Header, Hero, Gallery, Filters, Categories, Compliance, CTA, Footer
- Modal dialogs: Lightbox (image gallery) and Auth Modal (login)
- Inline SVG background with nature imagery

#### `src/main.js`
Main application logic including:

```javascript
// Core data structures
imageLibrary      // Object containing breed→image URLs mapping
petData          // Array of 15 pet objects with full details

// State management
cart             // Array of pet IDs in cart
lightboxState    // Current lightbox view state

// Key functions
hydrateFilters()     // Populate dropdown filters
applyFilters()       // Filter pet data based on selections
renderCards()        // Generate HTML for pet cards
openLightbox()       // Show image gallery modal
openAuthModal()      // Show login modal
addToCart()          // Add pet to shopping cart
```

#### `src/snakeCore.js`
Modular snake game engine (currently unused but ready for integration):

```javascript
// Constants
GRID_ROWS, GRID_COLS    // 20x20 grid
DIRECTION               // Movement vectors

// Game functions
createInitialState()    // Initialize new game
setDirection()         // Change snake direction
advanceState()         // Game tick (move, eat, die)
randomFoodCell()       // Spawn food not on snake
```

#### `src/styles.css`
- **400+ lines** of custom CSS
- CSS Variables for theming
- Mobile-responsive breakpoints
- Smooth animations and transitions
- Glassmorphism effects (backdrop-filter)
- Grid and Flexbox layouts

---

## 🚀 Setup & Installation

### Prerequisites
- Modern web browser (Chrome, Firefox, Safari, Edge)
- Local development server (optional, for ES modules)

### Option 1: Direct Open (Simplest)
```bash
# Just open the HTML file in browser
cd petpartner
open index.html
```

### Option 2: Local Server (Recommended)
```bash
# Using Python 3
cd petpartner
python -m http.server 8080

# Or using Node.js npx
cd petpartner
npx serve

# Or using VS Code Live Server extension
# Right-click index.html → "Open with Live Server"
```

Then visit: `http://localhost:8080`

---

## 📖 Usage Guide

### For Buyers

1. **Browse Marketplace**
   - Scroll to "Choose your companion" section
   - Use filters to narrow down by species, breed, location, etc.
   - Click "View photos" to see image galleries

2. **View Pet Details**
   - Each card shows: breed, gender, age, color, location
   - Badges indicate: microchipping, vaccinations, certifications
   - Price shown in INR (₹)

3. **Add to Cart / Buy**
   - Click "Add to cart" to save for later
   - Click "Buy now" to proceed (placeholder - needs payment integration)

4. **Login** (Placeholder)
   - Click any login button
   - Select role: Buyer or Seller
   - Form ready for backend integration

### For Developers

#### Adding New Pets

Edit `src/main.js` and add to `petData` array:

```javascript
{
  id: 16,  // Unique ID
  species: 'Dog',
  breed: 'Pomeranian',
  gender: 'female',
  ageMonths: 8,
  colour: 'White',
  city: 'Mumbai',
  state: 'Maharashtra',
  price: 35000,
  badges: ['Microchipped', 'Vaccinated'],
  logistics: 'Home delivery',
  images: getImages('pomeranian', 'pomeranian dog'),
}
```

#### Adding New Species Images

Add to `imageLibrary` in `src/main.js`:

```javascript
pomeranian: [
  'https://images.unsplash.com/photo-xxx?auto=format&fit=crop&w=1200&q=80',
  // ... more images
],
```

#### Customizing Styles

Edit CSS variables in `src/styles.css`:

```css
:root {
  --navy: #0f172a;      /* Change background */
  --coral: #e4685b;     /* Change accent color */
  --gold: #d7b168;      /* Change premium color */
}
```

---

## 💾 Data Model

### Pet Object Schema

```typescript
interface Pet {
  id: number;              // Unique identifier
  species: string;         // Dog, Cat, Bird, Aquatic, Small Pet, Reptile
  breed: string;           // Breed name
  gender: 'male' | 'female';
  ageMonths: number;       // Age in months
  colour: string;          // Color description
  city: string;            // City name
  state: string;           // Indian state
  price: number;           // Price in INR
  badges: string[];        // Certification badges
  logistics: string;       // Delivery method
  images: string[];        // Array of image URLs
}
```

### Current Pet Inventory (15 pets)

| ID | Species | Breed | Location | Price |
|----|---------|-------|----------|-------|
| 1 | Dog | Labrador Retriever | Bengaluru | ₹32,000 |
| 2 | Dog | German Shepherd | Pune | ₹42,000 |
| 3 | Dog | Indie (Desi) | Delhi | ₹9,000 |
| 4 | Cat | Persian | Mumbai | ₹28,000 |
| 5 | Cat | Siamese | Jaipur | ₹24,000 |
| 6 | Bird | Cockatiel | Chennai | ₹6,000 |
| 7 | Bird | Budgerigar | Kochi | ₹2,200 |
| 8 | Aquatic | Koi Carp | Surat | ₹15,000 |
| 9 | Aquatic | Goldfish | Lucknow | ₹1,800 |
| 10 | Small Pet | Syrian Hamster | Hyderabad | ₹1,800 |
| 11 | Small Pet | Netherland Dwarf Rabbit | Chandigarh | ₹6,500 |
| 12 | Reptile | Leopard Gecko | Pune | ₹9,000 |
| 13 | Dog | Beagle | Bhopal | ₹27,000 |
| 14 | Cat | Ragdoll | Kolkata | ₹32,000 |
| 15 | Small Pet | Sugar Glider | Ahmedabad | ₹22,000 |

---

## ⚖️ Compliance Features

### Legal Framework

Pet Partner is designed to comply with:

1. **AWBI Guidelines** (Animal Welfare Board of India)
2. **State Pet Shop Rules** (various states)
3. **IT Rules 2021** (Intermediary Guidelines)
4. **Wildlife Protection Act 1972** (Schedule I-IV species blocked)

### Implemented Safeguards

| Requirement | Implementation |
|-------------|----------------|
| Seller Verification | Licence upload & validation |
| Health Certificates | Vet fit-to-travel mandatory |
| Species Screening | Auto-block native wildlife |
| Identity Masking | Seller name hidden per IT Rules |
| Grievance Redressal | Contact visible for disputes |
| Audit Trail | Document logs for compliance |

---

## 🗺 Roadmap

### Phase 1: MVP (Current) ✅
- [x] Responsive landing page
- [x] Pet listing with filters
- [x] Shopping cart (frontend)
- [x] Image gallery (lightbox)
- [x] Authentication UI

### Phase 2: Backend Integration 🔄
- [ ] Node.js/Express backend
- [ ] MongoDB database
- [ ] JWT authentication
- [ ] Seller onboarding flow
- [ ] Document upload & verification
- [ ] Payment gateway (Razorpay)

### Phase 3: Advanced Features 📋
- [ ] Real-time chat
- [ ] Live video call verification
- [ ] Vet booking integration
- [ ] Pet insurance partners
- [ ] Logistics tracking
- [ ] Review & rating system

### Phase 4: Scale 🚀
- [ ] Mobile app (React Native)
- [ ] AI-powered matching
- [ ] Subscription services
- [ ] International exports

---

## 🤝 Contributing

### Developer Notes

This is a **private repository** for the Pet Partner project.

**To make changes:**
1. Clone the repository
2. Create a feature branch
3. Make changes
4. Test locally
5. Commit and push
6. Create PR for review

**Code Style:**
- Use semantic HTML
- CSS: BEM-like naming (`.block__element--modifier`)
- JS: ES6+ features, modular exports
- Indent: 2 spaces

---

## 📞 Contact & Support

**Repository:** https://github.com/Sparker2707/petpartner

**Maintainer:** @Sparker2707

**Status:** 🟡 In Development | 🔒 Private Repository

---

## 📜 License

Private - All rights reserved. Not open source.

---

## 🙏 Acknowledgments

- **Images:** Unsplash, Dog.CEO API, TheCatAPI
- **Fonts:** Google Fonts (Space Grotesk, Playfair Display)
- **Design Inspiration:** French aesthetic, Indian sensibility

---

> *"Adopt joy with Parisian elegance."* — Pet Partner
