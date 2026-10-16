import { OAuth2Client } from "google-auth-library";
import { config } from "../../config/env";
import { GoogleProfile } from "../../config/passport";

// No clientSecret needed here: verifying an ID token the client already
// obtained from Google is a public-key signature check, not an OAuth
// exchange. Audience is pinned to our own web OAuth client ID — the Android
// app's google_sign_in is configured with that same ID as its serverClientId
// so the tokens it mints carry a matching `aud` claim.
const client = new OAuth2Client();

export async function verifyGoogleIdToken(idToken: string): Promise<GoogleProfile> {
  const ticket = await client.verifyIdToken({
    idToken,
    audience: config.google.clientId,
  });

  const payload = ticket.getPayload();
  if (!payload?.email) {
    throw new Error("Google ID token did not include an email address");
  }
  if (!payload.email_verified) {
    throw new Error("Google account email is not verified");
  }

  return {
    googleId: payload.sub,
    email: payload.email.toLowerCase(),
    name: payload.name ?? payload.email,
    avatarUrl: payload.picture,
  };
}
