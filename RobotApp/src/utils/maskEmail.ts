/**
 * Masks an email for display so the full address is never shown on a
 * shared kiosk screen, e.g. "jatin.patil@example.com" -> "ja***l@ex***.com".
 */
export function maskEmail(email: string): string {
  const [local, domain] = email.split("@");
  if (!local || !domain) return "***";

  const maskedLocal =
    local.length <= 2 ? `${local.charAt(0)}*` : `${local.slice(0, 2)}${"*".repeat(Math.min(local.length - 2, 3))}`;

  const domainParts = domain.split(".");
  const domainName = domainParts[0] ?? "";
  const tld = domainParts.slice(1).join(".");
  const maskedDomainName =
    domainName.length <= 2
      ? `${domainName.charAt(0)}*`
      : `${domainName.slice(0, 2)}${"*".repeat(Math.min(domainName.length - 2, 3))}`;

  return `${maskedLocal}@${maskedDomainName}${tld ? `.${tld}` : ""}`;
}
